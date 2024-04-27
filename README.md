# Custom-Robot-navigation-using-NAV2-and-ROS2-Humble


This project demonstrates custom robot navigation using ROS2 Humble and NAV2, incorporating SLAM for mapping and path planning. The robot utilizes Lidar sensor technology for accurate environmental perception.
## Installation

Make sure you have ROS2 Humble installed. You can follow the official installation instructions at https://docs.ros.org/en/humble/Installation.html

Install NAV2 for ROS2. You can follow the installation instructions  at https://navigation.ros.org/

Install SLAM Toolbox for ROS2. You can follow the installation instructions at https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

**Clone this repository into your ROS2 workspace and build the packages**

```bash
cd /path/your_ros2_workspace/src
git clone https://github.com/Shashwat1524/Custom-Robot-navigation-using-NAV2-and-ROS2-Humble.git .
cd ..
colcon build
```
## Usage

### Launch the robot in the Gazebo environment:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

This command launches the robot in Gazebo with the specified world.

### Launch NAV2 for navigation:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

This command launches NAV2, allowing the robot to perform mapping, localization, and path planning.

### Launch SLAM for mapping:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

This command initiates SLAM, enabling the robot to create a map of its environment as it moves.

### Control the robot using TurtleBot3 teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

This command allows you to control the robot's movements using keyboard inputs.

### Open RViz to visualize the map:
```bash
rviz2 rviz2
```

Add map and TFs to RViz to view the map being formed as you maneuver the robot.

### Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map_world
```

This command saves the generated map to a file. Replace my_map_world with the desired file name.

### Launch navigation:
```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=maps/my_map_world.yaml
```
This command launches NAV2 for navigation, providing it with the previously generated map.

Subscribe to the map topic in RViz to visualize the map.

## Navigating the Robot
Use 2D pose estimator and 2D goal pose in RViz for navigation.
Alternatively, utilize **nav2_test.py** to set the current and target poses, then run the Python script.
