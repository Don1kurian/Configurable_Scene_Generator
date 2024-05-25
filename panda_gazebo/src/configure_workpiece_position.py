#!/usr/bin/env python3
import sys
import time
import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from get_model_info import get_model_state, write_pose_to_sdf_file


def randomize_workpiece_position(position, yaw):
    x,y=position
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # # SET CUBES POSITION # # #
    model_name_model = "workpiece"  # Assign the model to a state
   
    model_state = ModelState()
    model_state.model_name = model_name_model
    model_state.pose = Pose()
    model_state.pose.position.x=x
    model_state.pose.position.y=y
    model_state.pose.position.z = uniform(0.205008, 0.205009)
    model_state.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
    try:
        set_state(model_state)
        time.sleep(0.1)  # Add a delay
        rospy.loginfo("workpiece position randomized.")
        
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo service: %s", str(e))

def find_coordinates(position_index):
    # Define the grid bounds
    x_bounds = (0.5, 0.7)
    y_bounds = (-0.3, 0.3)
    # Define the number of rows and columns
    rows = 3
    cols = 3
    # Calculate the step sizes for x and y
    x_step = (x_bounds[1] - x_bounds[0]) / (cols - 1)
    y_step = (y_bounds[1] - y_bounds[0]) / (rows - 1)

    # Calculate the row and column index based on the position index
    row = (position_index - 1) // cols
    col = (position_index - 1) % cols

    # Calculate the coordinates based on the row and column index
    x = x_bounds[0] + col * x_step
    y = y_bounds[0] + row * y_step

    return x, y


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python script.py <position_index> <yaw>")
        sys.exit(1)

    position_index = int(sys.argv[1])
    yaw = float(sys.argv[2])
    position=find_coordinates(position_index)
    rospy.init_node('randomize_holes')
    randomize_workpiece_position(position_index, yaw)
    workpiece_pose = get_model_state("workpiece")
    time.sleep(0.01)  # Add a delay
    write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf", workpiece_pose)