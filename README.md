# Differential Drive Robot (ddr) Project 🤖

Welcome to the **Differential Drive Robot (ddr)** project! This repository provides a comprehensive setup for building and simulating a differential drive robot using **ROS2** and **Nav2**. Whether you're a beginner or looking to enhance your robotics skills, this project will guide you through the essentials of ROS2 navigation.

## 🚀 Purpose

This project aims to help you:
- Understand the basics of ROS2 and its navigation stack (Nav2).
- Set up a simulated differential drive robot in Gazebo.
- Experiment with autonomous navigation and localization using AMCL.
- Control a camera mounted on the robot for enhanced observation.

## 🌟 Key Features

- **Easy Setup**: Step-by-step instructions to get your robot up and running quickly.
- **Autonomous Navigation**: Utilize the ROS2 Nav2 stack for effective path planning and obstacle avoidance.
- **Realistic Simulation**: A fully functional robot model in Gazebo Classic, complete with sensors and URDF descriptions.
- **AMCL Localization**: Automatically localize your robot in a pre-existing map.
- **Joystick-Controlled Camera**: Control the camera's yaw motion to observe surroundings while the robot navigates autonomously.
- **Scripts**: Simple scripts built for navigating to pose and following waypoints

## 📂 Repository Structure

Here's an overview of the directory structure:
```
ddr/
├── config/       # Configuration files for Nav2, robot parameters, and sensors
├── launch/       # Launch files for starting simulation, navigation, and localization
├── maps/         # Pre-built map file for localization
├── robot_desc/   # URDF files describing the robot model for Gazebo
├── src/          # Python scripts for controlling the robot and camera
└── README.md     # Project documentation
```

## 🎮 Getting Started

### Launching Your Robot

1. **Start Gazebo Simulation**:
Launch the simulation environment:
```ros2 launch ddr robot_gazebo_classic.launch.py```


2. **Launch Map Server and AMCL**:
Start both the map server and AMCL localization with one command:
```ros2 launch ddr map_and_localization.launch.py```

Then set the map topic and provide the initial pose of robot using the '2D Initial Pose' option in RVIZ


3. **Start Navigation Stack**:
After launching mapping and localization, start the navigation stack:
```ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true```


4. **Launch Twist Mux (for command handling)**:
Ensure proper command handling by launching twist mux:
```ros2 run twist_mux twist_mux --ros-args --params-file ./src/ddr/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped```
This command set the twist mux parameters by referring to the twist_mux.yaml file and remaps the cmd_vel_out topic which is default output topic of twist mux to diff_cont/cmd_vel_unstamped which is used by differential drive controller


5. **Control the Camera**:
To control the camera's yaw motion using a joystick:
```ros2 run ddr camera_motion_controller.py```

Make sure to connect the controller prior or you can use the following virtual joystick package:
https://github.com/openrosbotix/virtual_joystick


## 💡 Author's Note

I have tried to build this project to understand and experiment with ROS2 and Nav2. I tried tinkering with the ros2_control by creating a custon scipt to control the camera while robot is autonomously navigating. I have also added the script to navigate to particular pose and waypoint following in `src` folder which were made using the `nav2_simple_commander` package. 


## 🤝 Contributing

We welcome contributions! If you have suggestions, improvements, or bug fixes, please submit a pull request.

## Special Thanks 

Virtual Joytick : https://github.com/openrosbotix/virtual_joystick
Reference for urdf: https://github.com/joshnewans/articubot_one
