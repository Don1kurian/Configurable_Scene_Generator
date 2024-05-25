#!/usr/bin/env python3

import numpy as np
import time
import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState, DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from matrix_transform_to_world import transform_from_local_to_world
from get_model_info import get_model_dimensions, get_model_pose,get_model_state,set_dimensions_in_sdf, write_pose_to_sdf_file, read_original_sdf, delete_model, spawn_model


def randomize_welding_line():

    welding_line_model_name = "welding_line"
    welding_line_model_path = "/home/don/your_workspace/src/panda_gazebo/resources/models/welding_line/model.sdf"  # Replace with your hole model name
    welding_line_model_state= ModelState()
    welding_line_model_state.pose = Pose()
    welding_line_model_state.pose=get_model_state(welding_line_model_name)
    delete_model(welding_line_model_name)                                  # Delete the current model
    time.sleep(.25)  # Add a delay
    #Get dimensions of Workpiece
    workpiece_pose = get_model_pose("workpiece")
    workpiece_dimensions = get_model_dimensions("/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf")
    workpiece_length, workpiece_width, workpiece_height = workpiece_dimensions
    rospy.loginfo(f"workpiece_diamension: {workpiece_length},{workpiece_width}")
    time.sleep(.25)  # Add a delay

    # Choose whether welding line should be vertical or horizontal
    vertical_welding = np.random.choice([True, False])
    if vertical_welding:
        x_dimension_welding_line = workpiece_length
        welding_line_x = 0
        welding_line_y = uniform(-workpiece_width / 2, workpiece_width / 2)
        welding_line_z = uniform(0.00551, 0.00552)
    else:
        x_dimension_welding_line = workpiece_width
        welding_line_x = uniform(-workpiece_length / 2, workpiece_length / 2)  # Align along length
        welding_line_y = 0
        welding_line_z = uniform(0.00551, 0.00552)
    local_welding_line_cordinates = (welding_line_x,welding_line_y,welding_line_z)
    world_cordinates_weldingline =transform_from_local_to_world(local_welding_line_cordinates, workpiece_pose)
    rospy.loginfo(f"welding_line_position(center) in world_coordinates: {world_cordinates_weldingline}")
    
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    # # SET Welding line POSITION # # #
    
    welding_line_model_state.model_name = welding_line_model_name
    welding_line_model_state.pose.position.x = world_cordinates_weldingline[0]
    welding_line_model_state.pose.position.y = world_cordinates_weldingline[1]
    welding_line_model_state.pose.position.z = world_cordinates_weldingline[2]
    if(vertical_welding):
        welding_line_model_state.pose.orientation = Quaternion(*quaternion_from_euler(workpiece_pose[3],workpiece_pose[4],workpiece_pose[5]))
    else:
        welding_line_model_state.pose.orientation = Quaternion(*quaternion_from_euler(workpiece_pose[3],workpiece_pose[4],workpiece_pose[5]+1.5704))
    modified_sdf=write_pose_to_sdf_file(welding_line_model_path, welding_line_model_state.pose)
    modified_sdf = set_dimensions_in_sdf(welding_line_model_path,(x_dimension_welding_line,0.005,0.001))      # Set specific values for x, y, and z dimensions
    time.sleep(.25)  # Add a delay
    spawn_model(welding_line_model_name, modified_sdf, welding_line_model_state.pose)
    time.sleep(.25)  # Add a delay
    try:
        set_state(welding_line_model_state)
        time.sleep(1)  # Add a delay
        rospy.loginfo("Welding line position randomized.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo service: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('randomize_welding_line')
    randomize_welding_line()
