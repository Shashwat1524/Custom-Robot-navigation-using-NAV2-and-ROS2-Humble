# Custom-Robot-navigation-using-NAV2-and-ROS2-Humble


This project demonstrates custom robot navigation using ROS2 Humble and NAV2, incorporating SLAM for mapping and path planning. The robot utilizes Lidar sensor technology for accurate environmental perception.
## Installation

Make sure you have ROS2 Humble installed. You can follow the official installation instructions here.

Clone this repository into your ROS2 workspace:

'''
cd /path/to/your/ros2/workspace/src
git clone <repository_url>
'''

### Build the packages:

bash

    cd /path/to/your/ros2/workspace
    colcon build

#### Packages
**my_robot_description**

Contains the URDF description of the robot.
**my_robot_bringup**

Contains launch files for launching the robot in a Gazebo environment along with a custom world.
Getting Started

### Launch the robot in the Gazebo environment:

ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

This command launches the robot in Gazebo with the specified world.

### Launch NAV2 for navigation:

go

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

This command launches NAV2, allowing the robot to perform mapping, localization, and path planning.

### Launch SLAM for mapping:

go

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

This command initiates SLAM, enabling the robot to create a map of its environment as it moves.

Control the robot using TurtleBot3 teleop:

arduino

ros2 run turtlebot3_teleop teleop_keyboard

This command allows you to control the robot's movements using keyboard inputs.

### Open RViz to visualize the map:

rviz2

RViz is a visualization tool for ROS that allows you to visualize the robot's sensor data and navigation information.

Add map and TFs to RViz to view the map being formed as you maneuver the robot.

### Save the map:

arduino

ros2 run nav2_map_server map_saver_cli -f maps/my_map_world

This command saves the generated map to a file. Replace my_map_world with the desired file name.

### Launch navigation:

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=maps/first.yaml

This command launches NAV2 for navigation, providing it with the previously generated map.

Subscribe to the map topic in RViz to visualize the map.

## Usage

Use 2D pose estimator and 2D goal pose in RViz for navigation.
Alternatively, utilize nav2_test.py to set the current and target poses, then run the Python script.
