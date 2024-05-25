#!/usr/bin/env python3

import time
import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from matrix_transform_to_world import transform_from_local_to_world
from get_model_info import get_model_dimensions, get_model_pose, write_pose_to_sdf_file


def randomize_hole_position():
    model_name = "workpiece"
    model_path = "/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf" 
    workpiece_pose = get_model_pose(model_name)
    workpiece_dimensions = get_model_dimensions(model_path)
    length, width, height = workpiece_dimensions
    
    #Randomize three points in relative cordinate system respect to workpiece
    hole_1_x,hole_1_y,hole_1_z = uniform(-length/2+0.01, length/2-0.01), uniform(-width/2+0.01, width/2-0.01),uniform(0.005008,0.005009)
    hole_2_x,hole_2_y,hole_2_z = uniform(-length/2+0.01, length/2-0.01), uniform(-width/2+0.01, width/2-0.01),uniform(0.005008,0.005009)    
    hole_3_x,hole_3_y,hole_3_z = uniform(-length/2+0.01, length/2-0.01), uniform(-width/2+0.01, width/2-0.01),uniform(0.005008,0.005009)
   

    local_cordinates_hole_1 = (hole_1_x,hole_1_y,hole_1_z)
    local_cordinates_hole_2 = (hole_2_x,hole_2_y,hole_2_z)
    local_cordinates_hole_3 = (hole_3_x,hole_3_y,hole_3_z)

    world_cordinates_hole_1 =transform_from_local_to_world(local_cordinates_hole_1, workpiece_pose)
    world_cordinates_hole_2 =transform_from_local_to_world(local_cordinates_hole_2, workpiece_pose)
    world_cordinates_hole_3 =transform_from_local_to_world(local_cordinates_hole_3, workpiece_pose)
    rospy.loginfo(f"Hole_1_position in world_coordinates: {world_cordinates_hole_1}")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # # SET Holes POSITION # # #
    model_name_hole_1 = "hole_1"  # Replace with your hole model name
    hole_1_state = ModelState()
    hole_1_state.model_name = model_name_hole_1
    hole_1_state.pose = Pose()
    hole_1_state.pose.position.x = world_cordinates_hole_1[0]
    hole_1_state.pose.position.y = world_cordinates_hole_1[1]
    hole_1_state.pose.position.z = world_cordinates_hole_1[2]
    modified=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/hole_1/model.sdf", hole_1_state.pose)

    model_name_hole_2 = "hole_2"  # Replace with your hole model name
    hole_2_state = ModelState()
    hole_2_state.model_name = model_name_hole_2
    hole_2_state.pose = Pose()
    hole_2_state.pose.position.x = world_cordinates_hole_2[0]
    hole_2_state.pose.position.y = world_cordinates_hole_2[1]
    hole_2_state.pose.position.z = world_cordinates_hole_2[2]
    modified=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/hole_2/model.sdf", hole_2_state.pose)

    model_name_hole_3 = "hole_3"  # Replace with your hole model name
    hole_3_state = ModelState()
    hole_3_state.model_name = model_name_hole_3
    hole_3_state.pose = Pose()
    hole_3_state.pose.position.x = world_cordinates_hole_3[0]
    hole_3_state.pose.position.y = world_cordinates_hole_3[1]
    hole_3_state.pose.position.z = world_cordinates_hole_3[2]
    modified=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/hole_3/model.sdf", hole_3_state.pose)

    try:
        set_state(hole_1_state)
        time.sleep(1)  # Add a delay
        set_state(hole_2_state)
        time.sleep(1)  # Add a delay
        set_state(hole_3_state)
        time.sleep(1)  # Add a delay
        rospy.loginfo("Holes position randomized.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo service: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('randomize_holes')
    randomize_hole_position()
