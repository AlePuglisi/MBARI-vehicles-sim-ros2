# ROV-Ricketts-ros2 | COMMAND SHEET 
**Unofficial MBARI's ROV Simulation and Control Project** <br/>
Underwater World Simulation | ROV | ROS2 Jazzy | Gazebo Harmonic | 

<image width=450 heigth=300 src=https://github.com/user-attachments/assets/5a26e743-ca25-4e6b-828f-766e30c5d696>
  
### Clone this repository and build 
Move to your ros workspace, inside /src folder (where all your packages are located) 
```
git clone https://github.com/AlePuglisi/ROV-Ricketts-ros2.git
```

### Terminal Commands to launch visualization
- Visualize Ricketts **with the Robotic Arm** in Rviz: 
  ```
  ros2 launch rov_ricketts_description display.launch.py load_arm:=true
  ```
- Visualize Ricketts **without the Robotic Arm** in Rviz: 
  ```
  ros2 launch rov_ricketts_description display.launch.py load_arm:=false
  ```
  Or simply (being the default configuration set to ``false``): 
  ```
  ros2 launch rov_ricketts_description display.launch.py 
  ```
  
  <image width=500 heigth=300  src=https://github.com/user-attachments/assets/5c3c9c66-c7f4-4276-957b-2d1cf4b8ae2d>

### Terminal Commands to launch simulation

- Spawn the robot in the default world specified in ``rov_sim.launch.py``:
  ```
  ros2 launch rov_ricketts_sim rov_sim.launch.py 
  ```
- Spawn the robot in a chosen world:
  ```
  ros2 launch rov_ricketts_sim rov_sim.launch.py world:=<path/to/world.sdf>
  ```
  
  <img width=350 heigth=300 src=https://github.com/user-attachments/assets/0bf0a30f-8224-4e0a-95b1-83ac638adb4e>
  
  <img width=350 heigth=300 src=https://github.com/user-attachments/assets/031b6c89-4a83-423f-9603-b400da519277>

### Terminal Commands to interact with gazebo plugins:
 <img width=200 heigth=265 src=https://github.com/user-attachments/assets/1f8a6c79-3812-4c69-9b7b-5c60168da0e2>
 <img width=400 heigth=265 src=https://github.com/user-attachments/assets/89e07d87-b08a-4938-88d9-c57e42595556>

- Change body **buoyancy**:
  
  ```
  ros2 topic pub --once /rov_ricketts/buoyancy std_msgs/msg/Float64 "{data: <new_buoyancy>}"
  ```
- Control **thrusters**:
  
  ```
  ros2 topic pub --once /rov_ricketts/front_left_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  ```
  ros2 topic pub --once /rov_ricketts/front_right_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  ```
  ros2 topic pub --once /rov_ricketts/rear_left_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  ```
  ros2 topic pub --once /rov_ricketts/rear_right_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  ```
  ros2 topic pub --once /rov_ricketts/up_front_left_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  ```
  ros2 topic pub --once /rov_ricketts/up_front_rigth_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  ```
  ros2 topic pub --once /rov_ricketts/up_front_rear_thrust std_msgs/msg/Float64 "{data: <thrust_command>}"
  ```
  <image src=https://github.com/user-attachments/assets/d2cdf1e2-7910-4672-8660-4bd7bc90544e>

