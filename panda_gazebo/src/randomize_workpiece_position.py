#!/usr/bin/env python3

import time
import rospy
from random import uniform
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from get_model_info import get_model_state, write_pose_to_sdf_file


def randomize_workpiece_position():
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # # SET CUBES POSITION # # #
    model_name_model = "workpiece"  # Assign the model to a state
    min_x, max_x = 0.55, 0.65  # Adjust the desired range for x position
    min_y, max_y = -0.25, 0.25  # Adjust the desired range for y position
    min_z, max_z = 0.205008, 0.205009  # Adjust the desired range for z position
    min_c, max_c = 0.00, 3.14    # Adjust the desired rotational range
   
    model_state = ModelState()
    model_state.model_name = model_name_model
    model_state.pose = Pose()
            
    model_state.pose.position.x = uniform(min_x, max_x)
    model_state.pose.position.y = uniform(min_y, max_y)
    model_state.pose.position.z = uniform(min_z, max_z)
    model_state.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, uniform(min_c, max_c)))

    try:
        set_state(model_state)
        time.sleep(0.1)  # Add a delay
        rospy.loginfo("workpiece position randomized.")
        
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo service: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('randomize_holes')
    randomize_workpiece_position()
    workpiece_pose = get_model_state("workpiece")
    time.sleep(0.01)  # Add a delay
    write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf", workpiece_pose)