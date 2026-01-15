# MBARI Vehicles Simulation with Stonefish and Gazebo + ROS2
**Unofficial MBARI's ROV Simulation and Control Project** <br/>
I started this project because of my interest in underwater robotics and fascination with the [MBARI](https://www.mbari.org/) Oceanographic Institute. <br/>

## Implemented Simulations

- [Stonefish Simulation ROS2](#stonefish-simulation-ros2) | Under development
    - [MOLA AUV](#mola-auv)
    - [ROV Doc Ricketts](#rov-doc-ricketts)  
- [Gazebo Simulation ROS2](#gazebo-simulation-ros2) | Archived
    - [ROV Doc Ricketts](#rov-doc-ricketts)

## Stonefish Simulation ros2
Underwater World Simulation | ROV | ROS2 Jazzy | Stonefish | 

### MOLA AUV

### ROV Doc Ricketts


## Gazebo Simulation ros2
Underwater World Simulation | ROV | ROS2 Jazzy | Gazebo Harmonic | 

I'm no more updating and improving the simulation in Gazebo, since I switch to differenty simulators tailored to the underwater domain (such as stonefish). <br/>
Even if this part of the project is "archived", It still contain some valuable information to set up your own Gazebo simulation for a robotics project. <br/>

Moreover, from the moment in which I was working on this project (April 2025), Gazebo underwater sensors and plug-ins may have been upgraded ! 

### ROV Doc Ricketts
<image width=375 heigth=250 src=https://github.com/user-attachments/assets/5a26e743-ca25-4e6b-828f-766e30c5d696>
<img width=375 heigth=250 src=https://github.com/user-attachments/assets/89e07d87-b08a-4938-88d9-c57e42595556>

> [!IMPORTANT]
> To visualize the documentation of the commands only, look at this [commands sheet readme](https://github.com/AlePuglisi/ROV-Ricketts-ros2/blob/main/COMMANDS-SHEET.md). 

During this project, I want to learn how to set up an underwater world simulation with ROS2 and Gazebo. <br/>
Gazebo Classic has reached its end of life, and it is time to learn how to use Gazebo sim. <br/>
Due to some rendering and world dimension limitations of Gazebo, soon, I want to explore other robotics simulators, like [NVIDIA Omniverse IsaacSim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html). 

### Final (long-term) Objective
Thanks to the huge amount of available models of ROV, Environments, and Creatures gifted by MBARI on [sketchfab](https://sketchfab.com/search?q=mbari&type=models), I want to: 
- Create a URDF Model of MBARI's  [ROV Doc Ricketts](https://www.mbari.org/technology/rov-doc-ricketts/)
   - Model each thruster as an independent joint
   - Model and Integrate the actuated ROV's Manipulator (with all joints) <br/>
     (Even If currently Ricketts has 2 arms for advanced manipulation tasks, I will consider this only in the future) 
- Define an approximated fluid dynamic model of the ROV, for simulation
- Learn to correctly set up Gazebo sim plugins to simulate thrusters, buoyancy, and fluid dynamics force
- Create a custom sdf world file, with deep-sea environments from MBARI's 3D sea floor reconstruction 
- Add deep sea creatures as [Gazebo Actors](https://gazebosim.org/docs/latest/actors/), to make this world even more alive. <br/>
  This is possible thanks to [Photogrammetry Techniques](https://www.mbari.org/wp-content/uploads/Kaiser_Nicole.pdf) used by MBARI.
- Implement high and low-level ROS nodes to control and teleoperate the ROV motion (exploiting the 7 thrusters) 
- Implement high and low-level ROS nodes to control and teleoperate the ROV Arm (including Gripper operations)
- Add light sources, cameras, and other exteroceptive/proprioceptive sensors to the Robot Model. 
- Implement control on Pan-Tilt Camera motion
- Set up needed TFs, sensors, topics, and odometry to Enable Mapping with SLAM
- Set up Autonomous Navigation Features
- Implement high-level algorithms for Visual Tracking of deep water animals <br/>
  (Due to Gazebo's Limitations in rendering camera images, this may require moving to IsaacSim)
- Implement high-level algorithms for Visual Servo Control of the ROV Robotic Arm.
- ...

> [!TIP]
> The following steps can be useful for anyone who wants to create a Robotic Simulation from the robot mesh! <br/>
> Here world and robot plugins will be underwater specific, but the high-level concepts remain the same even for a ground vehicle. <br/>
> This is why I'll try to describe the implementation step as clearly as possible.<br/>
> Later on I will provide a concise command sheet to run the simulation and use its functionalities. 


... This is a lot just for a single person, :raised_hand_with_fingers_splayed: a big high five to anyone opening an issue for suggestions, ideas, or reviews. <br/>
Feel free to [Pull Requests](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) to contribute to this project. <br/>

This emoji " :point_right: "Indicates the current step in development. 

### Step 0. Install and Set up ROS2 + Used ROS Packages 
### Prerequisite
- Linux Ubuntu 24.04 
- ROS2 Jazzy
> [!NOTE]
> Alternatively, you can build a ``Docker`` container to work with ROS2 Jazzy in your OS.<br/>
> If you are using Docker, refer to this section just to know the required dependencies.

<details> 
<summary> Install and Set Up ROS2 Jazzy </summary>
   
- **Install** ROS Jazzy and source 
Refer to [ros documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for the step-by-step installation tutorial. <br/>

Anytime you open a new terminal, you should source your underlay ros environment with:
```
# Terminal
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```
To avoid doing it every time, add that command to your ``bashrc``, opening it with a text editor of your choice:
```
# Terminal (use your preferred text editor tool)
gedit ~/.bashrc
```
And add the line ``source /opt/ros/jazzy/setup.bash`` at the end.

- If not automatically installed, **install ``colcon``**, the build tool for ROS2: 
```
# Terminal
sudo apt install python3-colcon-common-extensions
```

- Create and build a **ros2 workspace** to position this project packages:<br/>
In your preferred directory
```
# Terminal
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
```

- **Source** your ros2 workspace and the colcon argmícomplete environment:<br/>
As done for the ros underlay, you need to source the overlay setup.bash when you build it, again you can add that to the bashrc
```
# Terminal
gedit ~/.bashrc
```
Add the line ``source ~/ros2_ws/install/setup.bash`` <br/>
and ``source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`` to complete colocn commands by pressing Tab.

### Install Additional Packages
To launch this project packages, you may need ROS2 packages not installed with ros-jazzy-desktop. <br/>
I will update this list during implementation:

- ``xacro``
- ``joint_state_publisher_gui``
- ...

```
# Terminal
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui 
```
</details>

### Clone this repository and build 
Move to your ros workspace, inside /src folder (where all your packages are located) 
```
# Terminal
git clone https://github.com/AlePuglisi/ROV-Ricketts-ros2.git
```
then, move back to the workspace folder (before /src), and build your workspace.<br/>
Now you are ready to test this project!<br/>
(Remember that after building the workspace, you need to source your bashrc, so your overlay and underlay will be sourced, just run ``source ~/.bashrc``)

### :point_right: Step 1. Set up The Simulation

### 1.1. Create ROV Ricketts URDF from Sketchfab Model
- First I download [Doc Ricketts ROV](https://sketchfab.com/3d-models/doc-ricketts-rov-def365ad47894a06b5f0fb2876795bf9) model from Sketchfab, in ``fbx`` format.
- Import it in Blender to create separate meshes (with texture) for each piece
- Export each model as ``dae`` (COLLADA format)<br/>

> [!TIP]
> I discovered that ``stl `` format does not support texture by default, setting it up may be possible but hard. <br/>
> [Here](https://github.com/AlePuglisi/ROV-Ricketts-ros2/tree/main/rov_ricketts_description/meshes/stl) you can find stl models, but for an astonishing visualization, rely on ``dae``.<br/>
> With the COLLADA format, textures are easy to load (just be sure the texture png file is in the correct referenced path inside dae XML file). 

- Create the ``URDF`` connecting and positioning each thruster, as a revolute joint (probably continuous joint will be used later on). <br/>
I used the ``xacro`` format, to make it cleaner and allow a simple customization at launch time. 
- Inertia property tuning will be done later on when setting up Gazebo simulation
- Creta a simple launcher [``display.launch.py``](https://github.com/AlePuglisi/ROV-Ricketts-ros2/blob/main/rov_ricketts_description/launch/display.launch.py) to run Rviz, robot_state_publisher, joint_state_publisher (and its GUI).
- Add relevant visualization objects in Rviz and save as [`` rov_monitor.rviz`` ](https://github.com/AlePuglisi/ROV-Ricketts-ros2/blob/main/rov_ricketts_description/config/rov_monitor.rviz). 

Finally, it is possible to visualize the robot in Rviz and move the thrusters with joint_state_publisher_gui: <br/>
```
# Terminal
ros2 launch rov_ricketts_description display.launch.py
```

<image src=https://github.com/user-attachments/assets/cbda4232-3072-4fbb-bc24-a6344e62501a>
   

The next step is to model and attach the robotic arm to Doc Ricketts.<br/>

###  1.2. Add ROV Manipulator to the URDF Model 
The arm is defined as a single piece in the 3D model, three options are possible to derive the URDF model:
- [ ] 1. Remodel it with simple cylinders and parallelepiped shapes, connected by revolute joints, directly in URDF.
- [ ] 2. Create a new 3D model from scratch, with separate links, using the original arm as a reference
- [x] 3. Use the original model and modify it to cut the links, then use this as URDF links

<br/>
<image src=https://github.com/user-attachments/assets/c58b649c-45cf-4ff8-883d-6c081f94382e>

   
I found the information about the robotic arm on [this interesting video](https://www.youtube.com/watch?v=BTdeXxaGfAs&t=303s&ab_channel=MBARI%28MontereyBayAquariumResearchInstitute%29) about ROV technologies at MBARI.<br/>
Thank you very much, Benjamin Erwin :pray:. <br/>
ROV Doc Ricketts is equipped with two robotic arms: 
- **Shilling TITAN 4 Manipulaotr** (T4): Strong Titanium arm, for heavy work but less precise.
- **Kraft TeleRobotics PREDATOR Manipulator**: Aluminum Arm, very dexterous, for precise manipulation. 
The combination of the two ensures a trade-off between precision and power. 

I only modeled the [T4 manipulator](https://www.technipfmc.com/media/hpkjrigr/titan-4-datasheet.pdf),  available in the 3D Model. <br/>
 (In the URDF, I include the **arm_tool** frame, because this will help when performing manipulation tasks.)

> [!TIP]
> A useful tag to define **gripper joints** is "mimic": <br/>
>
>     <joint name="${name}_link4_claw_left" type="revolute">
>     <mimic joint="${name}_link4_claw_right" multiplier="1" offset="0"/>
>     ....
> 
> This forces this joint to move around its rotation axis as the mimic joint times the multiplier.<br/>
> (joint_state = mimic_joint_state * multiplier) <br/>
> Changing the multiplier value we can set an inverse rotation or a transformation ratio.

> [!NOTE]
> The default gazebo physics solver skip mimic joint constraints.
> This is not a real issue, the gripper commands will be handled using
> a ros2_control ``position_controllers/GripperActionController``


Once Doc Ricketts Arm URDF has been defined, to load T4 arm, we can use a launch argument and the xacro conditional statement. <br/>

- To configure Ricketts **with the Robotic Arm**: 
```
# Terminal
ros2 launch rov_ricketts_description display.launch.py load_arm:=true
```
- To configure Ricketts **without the Robotic Arm**: 
```
# Terminal
ros2 launch rov_ricketts_description display.launch.py load_arm:=false
```
(Notice that the default value of load_arm is false, so argument initialization can be neglected in that case)

As can be seen, all joints have been articulated properly! <br/>
Brief **Joint actuation Demo**:mechanical_arm:: 

<image src=https://github.com/user-attachments/assets/5c3c9c66-c7f4-4276-957b-2d1cf4b8ae2d>

Joint limits and dynamic joints/body properties (Inertia, Center of mass, etc..) will be defined in the next steps...

### 1.3. Tune Model Inertia parameters
Inertial parameters of are fundamental for setting up Gazebo simulation (see the official documentation [tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)). <br/>
   - ``<origin>``: position of the CoM, with respect to link frame [m]
   - ``<mass>``: link mass [kg]
   - ``<inertia>``: 3x3 inertia tensor [kg*m^2]

 For a reliable simulation in Gazebo, we have to define the mass, inertia matrix, and center of mass position of each link. <br/>
 I have used [trimesh](https://trimesh.org/), a Python library to compute mesh properties such as volume, CoM position, and inertia. 

- **T4 Manipulator**: <br/>
Using Titanium density, we can then use the mesh volume to compute each link's mass and CoM position.
The Inertia matrix is normalized, so I had to multiply by mass/volume to bring it to the standard unit.
- **ROV Body and Propellers**: <br/>
The **propellers** are assumed to be made of aluminum (commonly used underwater because of corrosion resistance). <br/> 
For the **base**, modeled as a unique body, a constant density hypothesis was not possible. MBARI's website provides information about
its mass value. However, because of the complex mesh shape and discontinuous density distribution,``trimesh`` computation gives an unreliable inertia value. <br/>
For this reason, base inertia and CoM are computed using a simple box shape of uniform density.<br/>
Furthermore, base ``<collision>`` is simplified to reduce simulation complexity, using the same box as the inertial one. 
<img width=300 heigth=200 src=https://github.com/user-attachments/assets/ad94bc53-9f3f-4a74-b00f-8a6e879ca8b9>
<img width=300 heigth=200 src=https://github.com/user-attachments/assets/9aa45b22-cd74-4720-b734-c7d4c19a2181>
<img width=300 heigth=200 src=https://github.com/user-attachments/assets/05cc1fa0-e875-42f7-9da0-2c02a59ecda1>



### 1.4. Add and Tune Buoyancy, fluid dynamic, thruster actuators, and ligths Gazebo sim plugins

### 1.5. Set up an underwater World in Gazebo Harmonic
<img width=470 heigth=400 src=https://github.com/user-attachments/assets/0bf0a30f-8224-4e0a-95b1-83ac638adb4e>
<img width=470 heigth=400 src=https://github.com/user-attachments/assets/031b6c89-4a83-423f-9603-b400da519277>

```
# Terminal
ros2 launch rov_ricketts_sim rov_sim.launch.py 
```

### To select the world sdf:

It is possible to use the "world:=" launch argument
The first figure on the left comes from **ocean.sdf** load also the ocean surface:
```
# Terminal
ros2 launch rov_ricketts_sim rov_sim.launch.py world:=~/ros2_ws/src/ROV-Ricketts-ros2/rov_ricketts_sim/worlds/ocean.sdf 
```

To spawn directly the robot in the underwater canyon, load it in **underwater_world.sdf** world:
```
# Terminal
ros2 launch rov_ricketts_sim rov_sim.launch.py world:=~/ros2_ws/src/ROV-Ricketts-ros2/rov_ricketts_sim/worlds/underwater_world.sdf 
```

<img width=450 heigth=300 src=https://github.com/user-attachments/assets/89e07d87-b08a-4938-88d9-c57e42595556>



### 1.6. Move ROV Ricketts in the Gazebo World

I'm not yet a good ROV pilot!
Adjustments on plugin parameters and a GUI or joypad teleoperation will be implemented to make Ricketts easy to operate.

<image src=https://github.com/user-attachments/assets/d2cdf1e2-7910-4672-8660-4bd7bc90544e>

### ... Archived ... 
I'm integrating with Stonefish the other features that I was planning to integrate in Gazebo...
