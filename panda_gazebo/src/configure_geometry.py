#!/usr/bin/env python3
import sys
import time
import rospy
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel
from get_model_info import get_model_state,set_dimensions_in_sdf

def spawn_model(model_name, model_xml,Pose,reference_frame="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    try:
        spawn_model_proxy(model_name, model_xml, "", Pose, reference_frame)
        rospy.loginfo(f"Model '{model_name}' spawned successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn model '{model_name}': {str(e)}")

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    try:
        delete_model_proxy(model_name)
        rospy.loginfo(f"Model '{model_name}' deleted successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to delete model '{model_name}': {str(e)}")


if __name__ == '__main__':
    rospy.init_node('set_dimensions_in_model')

    model_name = "workpiece"
    model_path = "/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf" 
    model_state= ModelState()
    model_state.pose=get_model_state(model_name)

    if len(sys.argv) != 3:
        print("Usage: rosrun your_package_name configure_workpiece.py <length> <width>")
        sys.exit(1)

    try:
        x_dimension = float(sys.argv[1])
        y_dimension = float(sys.argv[2])
        z_dimension = 0.01                       # z diamension is fixed as 1cm
        rospy.loginfo(f"New Dimension {x_dimension}'x'{y_dimension}")
    except ValueError:
        print("Error: Length and width must be numeric values")
        sys.exit(1)
    model_dimension=(x_dimension, y_dimension, z_dimension)
    rospy.loginfo(f"New Dimension of '{model_name}': {x_dimension}'x'{y_dimension}")
   
    modified_sdf = set_dimensions_in_sdf(model_path,model_dimension)      # Set specific values for x, y, and z dimensions
    time.sleep(.1)  # Add a delay
   
    delete_model(model_name)                                  # Delete the current model
    time.sleep(.1)  # Add a delay
    spawn_model(model_name, modified_sdf, model_state.pose)
    time.sleep(1)  # Add a delay