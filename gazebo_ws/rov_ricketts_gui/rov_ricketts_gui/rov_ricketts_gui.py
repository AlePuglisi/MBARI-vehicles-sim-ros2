import tkinter as tk
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
from ament_index_python.packages import get_package_share_directory
import os

class ROS2Node(Node):
    """ROS2 Node for publishing messages on button clicks."""
    def __init__(self):
        super().__init__("rov_ricketts_gui")
        self.get_logger().info('rov_ricketts_gui Initialized')

        # Create publisher for each thruster
        self.command_publishers = {
            "buoyancy": self.create_publisher(Float64, "/rov_ricketts/buoyancy", 10),
            "front_left_thrust": self.create_publisher(Float64, "/rov_ricketts/front_left_thrust", 10),
            "front_right_thrust": self.create_publisher(Float64, "/rov_ricketts/front_right_thrust", 10),
            "rear_left_thrust": self.create_publisher(Float64, "/rov_ricketts/rear_left_thrust", 10),
            "rear_right_thrust": self.create_publisher(Float64, "/rov_ricketts/rear_right_thrust", 10),
            "up_front_left_thrust": self.create_publisher(Float64, "/rov_ricketts/up_front_left_thrust", 10),
            "up_front_right_thrust": self.create_publisher(Float64, "/rov_ricketts/up_front_right_thrust", 10),
            "up_front_rear_thrust": self.create_publisher(Float64, "/rov_ricketts/up_front_rear_thrust", 10),
        }
    
    def publish_message(self, topic, value):
        try:
            msg = Float64()
            msg.data = float(value)
            self.command_publishers[topic].publish(msg)
            self.get_logger().info(f"Published {msg.data} to {topic}")
        except ValueError:
            self.get_logger().error(f"Invalid input for {topic}: {value}")


class GUIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Rov-Ricketts GUI")
        
        # Set up ROS 2 node in a separate thread
        rclpy.init()
        self.ros_node = ROS2Node()
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()
        
        # Set up GUI layout
        self.left_frame = tk.Frame(root, width=1000, height=1130, bg="#98bfef", bd=0, highlightthickness=0)
        self.left_frame.pack(side="left", fill="both", expand=True)
        
        self.right_frame = tk.Frame(root, width=400, height=1130, bg="white", bd=0, highlightthickness=0)
        self.right_frame.pack(side="right", fill="both", expand=True)
        
        # Canvas for background image
        self.canvas = tk.Canvas(self.left_frame, bg="#98bfef", width=1000, height=1130)
        self.canvas.pack(fill="both", expand=True)
        
        # Load background image
        package_path = os.path.join(get_package_share_directory('rov_ricketts_gui'), 'resources')
        bg_image_path = os.path.join(package_path, "Rov_Ricketts.png")
        self.load_background_image(bg_image_path)
        
        # Button and entry field definitions
        self.controls = [
            {"topic": "buoyancy", "image": "thruster.png", "x": 90, "y": 230},
            {"topic": "front_left_thrust", "image": "thruster.png", "x": 260, "y": 60},
            {"topic": "front_right_thrust", "image": "thruster.png", "x": 90, "y": 900},
            {"topic": "rear_left_thrust", "image": "thruster.png", "x": 260, "y": 1070},
            {"topic": "rear_right_thrust", "image": "thruster.png", "x": 1040, "y": 900},
            {"topic": "up_front_left_thrust", "image": "thruster.png", "x": 870, "y": 1070},
            {"topic": "up_front_right_thrust", "image": "thruster.png", "x": 1040, "y": 230},
            {"topic": "up_front_rear_thrust", "image": "thruster.png", "x": 870, "y": 60},
        ]
        
        self.entries = {}  # Store entry widgets
        self.create_buttons(package_path)
        
        # Right frame label
        self.info_label = tk.Label(self.right_frame, text="Information", bg="white", font=("Arial", 14))
        self.info_label.pack(pady=20)

    def load_background_image(self, image_path):
        """Load and display the background image on the canvas."""
        self.background_image = Image.open(image_path).resize((1000, 1130), Image.Resampling.LANCZOS)
        self.background_image_tk = ImageTk.PhotoImage(self.background_image)
        self.canvas.create_image(0, 0, anchor="nw", image=self.background_image_tk)
# Define a slightly darker color than #98bfef
    

    def create_buttons(self, package_path):
        border_color = "#7aa6d8"  # Adjust as needed
        """Create buttons with images and associated entry fields."""
        for control in self.controls:
            img_path = os.path.join(package_path, control["image"])
            button_image = Image.open(img_path).resize((70, 70), Image.Resampling.LANCZOS)
            button_image_tk = ImageTk.PhotoImage(button_image)
            
            # Entry field
            entry = tk.Entry(self.canvas, width=10, font=("Arial", 8))
            self.entries[control["topic"]] = entry
            
            # Button
            button = tk.Button(
                self.canvas,
                image=button_image_tk,
                command=lambda topic=control["topic"]: self.on_button_click(topic),
                bd=0,  # Remove extra border thickness
                relief="flat",  # Remove 3D effect that could create a border
                highlightbackground=border_color,  # Border color
                highlightthickness=2,  # Set thickness of highlight border
                background="#98bfef",  # Ensure button blends with the canvas
                activebackground=border_color,  # Button color when clicked
)

            button.image = button_image_tk  # Prevent garbage collection
            
            # Place widgets on the canvas
            self.canvas.create_window(control["x"], control["y"], anchor="nw", window=button)
            self.canvas.create_window(control["x"] - 30, control["y"] + 80, anchor="nw", window=entry)
        
    def on_button_click(self, topic):
        """Handle button click events."""
        value = self.entries[topic].get()
        self.ros_node.publish_message(topic, value)
        self.info_label.config(text=f"Published {value} to {topic}")


def main():
    root = tk.Tk()
    app = GUIApp(root)
    root.geometry("1400x1130")
    
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
