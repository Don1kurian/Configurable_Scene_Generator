#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
import math
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from get_model_info import get_model_dimensions
from matrix_transform_to_world import transform_from_local_to_world
tau = 2 * math.pi

def hoverPose(move_group, x, y, z):
    # We can plan a motion for this group to a desired pose for the end-effector.
    target_pose_hover = Pose()

    # Convert Orientation from RPY to Quaternion
    quaternion = Quaternion(*quaternion_from_euler(-tau/2, 0, 0))

    target_pose_hover.orientation = quaternion
    target_pose_hover.position.x = x
    target_pose_hover.position.y = y
    target_pose_hover.position.z = z
    move_group.set_pose_target(target_pose_hover)
    move_group.go()

def initPose(move_group):
    # Get the current robot state
    current_state = move_group.get_current_state()
    # Get the current joint values for the group
    joint_group_positions = current_state.joint_state.position

    # Set the desired joint positions for the initial pose
    joint_group_positions = [0, -tau / 8, 0, -3 * tau / 8, 0, tau / 4 + 0.03, tau / 8]
    # joint_group_positions[0] = 0.0
    # joint_group_positions[1] = -tau / 8  # -1/8 turn in radians
    # joint_group_positions[2] = 0.0
    # joint_group_positions[3] = -3 * tau / 8  # -8/8 turn in radians
    # joint_group_positions[4] = 0.0
    # joint_group_positions[5] = tau / 4 + 0.03  # 1/4 turn in radians
    # joint_group_positions[6] = tau / 8  # 1/8 turn in radians
    # Set the joint value target
    move_group.set_joint_value_target(joint_group_positions)
    # Set maximum velocity and acceleration scaling factors
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)
    # Move the robot to the initial pose
    move_group.go()


def pickPose(move_group, direction, x, y, z):
    if direction == "down":
        z -= 0.059
    elif direction == "up":
        z += 0.045

    pose_target = Pose()
    pose_target.orientation = Quaternion(*quaternion_from_euler(-tau/2, 0, 0))
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    move_group.set_pose_target(pose_target)
    move_group.go()

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('own_pick_place_V4', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    group_arm = moveit_commander.MoveGroupCommander(group_name)
    group_arm.set_planner_id("RRTConnect")
    group_arm.set_max_velocity_scaling_factor(0.05)
    group_arm.set_max_acceleration_scaling_factor(0.2)
    group_arm.set_planning_time(5.0)

    # Get current position from Gazebo
    model_name = "welding_line"
    response=ModelState()
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    response = get_model_state(model_name, "world")
    x, y, z = response.pose.position.x, response.pose.position.y, response.pose.position.z
    yaw=yaw = math.atan2(2 * (response.pose.orientation.w * response.pose.orientation.z), 1 - 2 * (response.pose.orientation.z ** 2))
    
    dimension= get_model_dimensions("/home/don/your_workspace/src/panda_gazebo/resources/models/welding_line/model.sdf")
    length, width, height = dimension
    rospy.loginfo("Length of the welding line: {} {}".format(length,yaw))
    
    # Calculate the start & end position of the line segment
    start_x,start_y,start_z = transform_from_local_to_world((-length/2,0.0,0.0),(x,y,z,0,0,yaw))
    end_x, end_y, end_z = transform_from_local_to_world((length/2,0.0,0.0),(x,y,z,0,0,yaw))
    
    rospy.loginfo("Start position (x, y, z): ({}, {}, {})".format(start_x, start_y, start_z))
    rospy.loginfo("End position (x, y, z): ({}, {}, {})".format(end_x, end_y, end_z))
    
    # Perform welding operation
    hoverPose(group_arm, start_x, start_y, start_z + 0.13)
    hoverPose(group_arm, end_x, end_y, end_z + 0.13)
    rospy.sleep(2.0)  # Delay for 2 seconds
    initPose(group_arm) 
    rospy.sleep(3.0)  # Delay for 3 seconds
    rospy.logwarn("Round end")

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
