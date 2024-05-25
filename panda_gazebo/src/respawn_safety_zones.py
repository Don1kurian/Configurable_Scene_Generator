#!/usr/bin/env python3

import time
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from get_model_info import get_model_dimensions, get_model_state, set_dimensions_in_sdf, write_pose_to_sdf_file


def delete_existing_model(model_name):
    model_state = get_model_state(model_name)
    if model_state is not None:
        
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        try:
            delete_model_proxy(model_name)
            rospy.loginfo(f"Model '{model_name}' deleted successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to delete model '{model_name}': {str(e)}")
    else:
        rospy.loginfo(f"Model '{model_name}' does not exist")

def spawn_model(model_name, model_xml, pose, reference_frame="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    try:
        spawn_model_proxy(model_name, model_xml, " ", pose, reference_frame)
        rospy.loginfo(f"Model '{model_name}' spawned successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn model '{model_name}': {str(e)}")

if __name__ == '__main__':
    rospy.init_node('respawn_safety_zones')
    workpiece_dimension = get_model_dimensions("/home/don/your_workspace/src/panda_gazebo/resources/models/workpiece/model.sdf")
  
    delete_existing_model("safety_green")                                                    # Delete the existing model
    modified_sdf_content=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/safety_green/model.sdf",get_model_state("workpiece"))
    time.sleep(1)  # Add a delay
    modified_sdf_content_2=set_dimensions_in_sdf("/home/don/your_workspace/src/panda_gazebo/resources/models/safety_green/model.sdf",modified_sdf_content, workpiece_dimension)
    spawn_model("safety_green",modified_sdf_content_2, get_model_state("workpiece"))         # Spawn the modified model with the updated dimensions at the current pose
    time.sleep(1)  # Add a delay

    delete_existing_model("danger_red_1")                                                    # Delete the existing model
    modified_sdf_content=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/danger_red_1/model.sdf",get_model_state("hole_1"))
    spawn_model("danger_red_1",modified_sdf_content, get_model_state("hole_1"))              # Spawn the modified model with the updated dimensions at the current pose
    time.sleep(1)  # Add a delay

    delete_existing_model("danger_red_2")                                                    # Delete the existing model
    modified_sdf_content=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/danger_red_2/model.sdf",get_model_state("hole_2"))
    spawn_model("danger_red_2",modified_sdf_content, get_model_state("hole_2"))              # Spawn the modified model with the updated dimensions at the current pose
    time.sleep(1)  # Add a delay

    delete_existing_model("danger_red_3")                                                    # Delete the existing model
    modified_sdf_content=write_pose_to_sdf_file("/home/don/your_workspace/src/panda_gazebo/resources/models/danger_red_3/model.sdf",get_model_state("hole_3"))
    spawn_model("danger_red_3",modified_sdf_content, get_model_state("hole_3"))              # Spawn the modified model with the updated dimensions at the current pose
    time.sleep(1)  # Add a delay