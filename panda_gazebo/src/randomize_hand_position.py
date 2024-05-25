#!/usr/bin/env python3
import math
import time
import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from get_model_info import get_model_state, write_pose_to_sdf_file,get_model_dimensions

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)


def randomize_hands_position():
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    hands_name_model = "hand_1"  # Assign the model to a state
    #Table location end points
    min_x, max_x = 0.4, 0.8  # Adjust the desired range for x position
    min_y, max_y = -0.3, 0.3  # Adjust the desired range for y position
    min_z, max_z = 0.200508, 0.200509  # Adjust the desired range for z position
  

    workpiece_pose = get_model_state("workpiece")
    workpiece_cordinates=(workpiece_pose.position.x,workpiece_pose.position.y,workpiece_pose.position.z)
    workpiece_dimensions = get_model_dimensions("/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf")
    max_dimension= max(workpiece_dimensions[0],workpiece_dimensions[1])
    hands_model_state = ModelState()
    hands_model_state.model_name = hands_name_model
    hands_model_state.pose = Pose()
    
    while True:       
        hands_model_state.pose.position.x = uniform(min_x, max_x)
        hands_model_state.pose.position.y = uniform(min_y, max_y)
        hands_model_state.pose.position.z = uniform(min_z, max_z)
        hand_cordinates=(hands_model_state.pose.position.x,hands_model_state.pose.position.y,hands_model_state.pose.position.z)
        hands_model_state.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        
        if calculate_distance(workpiece_cordinates, hand_cordinates)>((max_dimension+0.2)*0.7):
            break
    modified=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/hand_1/model.sdf", hands_model_state.pose)    
    try:
        set_state(hands_model_state)
        time.sleep(0.1)  # Add a delay
        rospy.loginfo("hand_1 position randomized.") 
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo service: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('randomize_hands_position')
    randomize_hands_position()
    
    