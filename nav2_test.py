#!/usr/bin/env python3
import rclpy 
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    #create quarternion
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose= PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp=navigator.get_clock().now().to_msg()
    pose.pose.position.x=position_x
    pose.pose.position.y=position_y
    pose.pose.position.z=0.0
    pose.pose.orientation.x=q_x
    pose.pose.orientation.y=q_y
    pose.pose.orientation.z=q_z
    pose.pose.orientation.x=q_w
    return pose
    

def main():

    # Initialize ROS2
    rclpy.init()
    nav = BasicNavigator()


    # Create initial pose
    initial_pose = create_pose_stamped(nav, 0.0,0.0,0.0)
    # Set initial pose
    #nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()


    #send nav_2 goal
    goal_pose1= create_pose_stamped(nav, 3.0 ,0.0 ,0.0)
    goal_pose2= create_pose_stamped(nav, -1.0 ,-2.0 ,0.0)
    goal_pose3= create_pose_stamped(nav, 2.0 ,-2.0 ,0)


    # nav.goToPose(goal_pose)
    waypoints = [goal_pose1, goal_pose2,goal_pose3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback=nav.getFeedback()


    # while not nav.isTaskComplete():
    #     feedback=nav.getFeedback()
    
    print(nav.getResult())

    # Wait until Nav2 becomes active

    # Shutdown ROS2
    rclpy.shutdown()
     
if __name__ == '__main__':
    main()
