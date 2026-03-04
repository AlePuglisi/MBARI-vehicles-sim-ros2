#!/usr/bin/env python3
import threading
import collections
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure

import matplotlib
matplotlib.use('TkAgg')          
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

try:
    from stonefish_ros2.msg import DVL
    HAS_DVL = True
except ImportError:
    HAS_DVL = False
    print('[sensor_plotter] WARNING: stonefish_ros2 not found – DVL disabled')

# GUI Parameters
WINDOW_SECONDS  = 30        # seconds of history shown in the rolling window
MAX_POINTS      = 3000      # ring-buffer capacity (samples)
UPDATE_INTERVAL = 0.05      # matplotlib refresh period (s)  = 20fps

# Physical Estimation Parameters
ATM_PRESSURE    = 101325.0  # Pa
RHO_WATER       = 1025.0    # kg/m³
GRAVITY         = 9.81      # m/s²



class RingBuffer:
    """Fixed-length deque that also tracks the wall-clock time of each sample."""
    def __init__(self, maxlen=MAX_POINTS):
        self.t = collections.deque(maxlen=maxlen)
        self.data = {}           # channel_name = deque

    def add_channels(self, *names):
        for n in names:
            self.data[n] = collections.deque(maxlen=MAX_POINTS)

    def push(self, t, **kwargs):
        self.t.append(t)
        for k, v in kwargs.items():
            self.data[k].append(v)

    def arrays(self):
        t = np.array(self.t)
        d = {k: np.array(v) for k, v in self.data.items()}
        return t, d


class SensorPlotterNode(Node):

    def __init__(self):
        super().__init__('sensor_plotter')
        self._lock = threading.Lock()
        self._t0   = None                  # first timestamp (for relative x-axis)

        # ring buffers 
        self.imu_buf = RingBuffer()
        self.imu_buf.add_channels('ax', 'ay', 'az', 'wx', 'wy', 'wz')

        self.dvl_buf = RingBuffer()
        self.dvl_buf.add_channels('vx', 'vy', 'vz', 'alt')

        self.pres_buf = RingBuffer()
        self.pres_buf.add_channels('depth')

        # ROS subscriptions 
        self.create_subscription(
            Imu, '/mola_auv/navigator/imu', self._imu_cb, 10)
        self.create_subscription(
            FluidPressure, '/mola_auv/navigator/pressure', self._pres_cb, 10)
        if HAS_DVL:
            self.create_subscription(
                DVL, '/mola_auv/navigator/dvl', self._dvl_cb, 10)

        self.get_logger().info('sensor_plotter ready – opening plot window …')

    # helpers

    def _stamp(self, header):
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    # Message callbacks

    def _imu_cb(self, msg: Imu):
        t = self._stamp(msg.header)
        with self._lock:
            self.imu_buf.push(
                t,
                ax=msg.linear_acceleration.x,
                ay=msg.linear_acceleration.y,
                az=msg.linear_acceleration.z,
                wx=msg.angular_velocity.x,
                wy=msg.angular_velocity.y,
                wz=msg.angular_velocity.z,
            )

    def _dvl_cb(self, msg):
        t = self._stamp(msg.header)
        with self._lock:
            self.dvl_buf.push(
                t,
                vx=msg.velocity.x,
                vy=msg.velocity.y,
                vz=msg.velocity.z,
                alt=msg.altitude,
            )

    def _pres_cb(self, msg: FluidPressure):
        t = time.time()
        if self._t0 is None:
            self._t0 = t
        t -= self._t0
        gauge = msg.fluid_pressure # - ATM_PRESSURE
        depth = gauge / (RHO_WATER * GRAVITY)
        with self._lock:
            self.pres_buf.push(t, depth=depth)


#  Matplotlib UI 

def build_figure():
    """Create figure, axes, and line objects. Returns (fig, axes_dict, lines_dict)."""
    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor('#1e1e2e')
    fig.suptitle('MOLA AUV – Live Sensor Monitor', color='white',
                 fontsize=14, fontweight='bold', y=0.98)

    gs = gridspec.GridSpec(4, 2, figure=fig,
                           hspace=0.55, wspace=0.35,
                           left=0.07, right=0.97,
                           top=0.94, bottom=0.06)

    PANEL_BG   = '#2a2a3e'
    GRID_COLOR = '#3a3a5a'

    palette = {
        'x':   '#ff6b6b',
        'y':   '#6bcfff',
        'z':   '#6bff9e',
        'alt': '#ffd06b',
    }

    axes = {}
    lines = {}

    def make_ax(gs_loc, title, ylabel, channels, labels, ylims=None):
        ax = fig.add_subplot(gs_loc)
        ax.set_facecolor(PANEL_BG)
        ax.set_title(title, color='white', fontsize=9, pad=4)
        ax.set_ylabel(ylabel, color='#aaaacc', fontsize=8)
        ax.set_xlabel('time (s)', color='#aaaacc', fontsize=8)
        ax.tick_params(colors='#aaaacc', labelsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor('#555577')
        ax.grid(True, color=GRID_COLOR, linewidth=0.5, linestyle='--', alpha=0.6)
        if ylims:
            ax.set_ylim(*ylims)
        lns = {}
        for ch, lbl in zip(channels, labels):
            color = palette.get(ch.lstrip('abcdefghijklmnopqrstuvwxyz_').lstrip('0123456789')
                                or ch[-1], '#cccccc')
            # simpler: map last char
            color = {'x': palette['x'], 'y': palette['y'],
                     'z': palette['z'], 't': palette['alt'],
                     'h': palette['alt']}.get(ch[-1], '#cccccc')
            ln, = ax.plot([], [], color=color, linewidth=1.2, label=lbl)
            lns[ch] = ln
        ax.legend(loc='upper left', fontsize=7,
                  facecolor=PANEL_BG, edgecolor='#555577',
                  labelcolor='white', framealpha=0.8)
        return ax, lns

    # Row 0 – IMU linear acceleration
    axes['acc'], lines['acc'] = make_ax(
        gs[0, :], 'IMU – Linear Acceleration', 'm/s²',
        ['ax', 'ay', 'az'], ['ax', 'ay', 'az'])

    # Row 1 – IMU angular velocity
    axes['gyro'], lines['gyro'] = make_ax(
        gs[1, :], 'IMU – Angular Velocity', 'rad/s',
        ['wx', 'wy', 'wz'], ['ωx', 'ωy', 'ωz'])

    # Row 2 – DVL velocity (left) + altitude (right)
    axes['dvl_v'], lines['dvl_v'] = make_ax(
        gs[2, 0], 'DVL – Velocity', 'm/s',
        ['vx', 'vy', 'vz'], ['vx', 'vy', 'vz'])

    axes['dvl_alt'], lines['dvl_alt'] = make_ax(
        gs[2, 1], 'DVL – Altitude', 'm',
        ['alt'], ['altitude'], ylims=None)

    # Row 3 – Depth
    axes['depth'], lines['depth'] = make_ax(
        gs[3, :], 'Pressure Sensor – Depth', 'm',
        ['depth'], ['depth'])
    # invert y so deeper = lower on screen
    axes['depth'].invert_yaxis()

    return fig, axes, lines


def update_ax(ax, line_dict, buf, window=WINDOW_SECONDS):
    """Refresh a set of lines from a RingBuffer. Returns True if data exists."""
    t, d = buf.arrays()
    if len(t) == 0:
        return False

    t_max = t[-1]
    t_min = max(t[0], t_max - window)
    mask  = t >= t_min

    t_win = t[mask]
    ax.set_xlim(t_min, t_min + window)

    for ch, ln in line_dict.items():
        if ch in d and len(d[ch]) == len(t):
            ln.set_data(t_win, d[ch][mask])

    ax.relim()
    ax.autoscale_view(scalex=False, scaley=True)
    return True


def run_plot(node: SensorPlotterNode):
    """Main-thread plotting loop (must run on main thread for Tk/Qt)."""
    fig, axes, lines = build_figure()
    plt.ion()
    plt.show(block=False)

    while rclpy.ok():
        with node._lock:
            update_ax(axes['acc'],     lines['acc'],     node.imu_buf)
            update_ax(axes['gyro'],    lines['gyro'],    node.imu_buf)
            update_ax(axes['dvl_v'],   lines['dvl_v'],   node.dvl_buf)
            update_ax(axes['dvl_alt'], lines['dvl_alt'], node.dvl_buf)
            update_ax(axes['depth'],   lines['depth'],   node.pres_buf)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(UPDATE_INTERVAL)

        if not plt.fignum_exists(fig.number):
            # shut down ROS cleanly
            rclpy.shutdown()
            break


def main(args=None):
    rclpy.init(args=args)
    node = SensorPlotterNode()

    # Spin ROS callbacks in a background thread so the main thread
    # stays free for Matplotlib (required by most GUI backends)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()