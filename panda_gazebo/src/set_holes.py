#!/usr/bin/env python3

import time
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from random import uniform

def set_hole_position():
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    workpiece_model_name = "workpiece"
    hole_model_name = "hole"

    # Get the current state of the workpiece
    workpiece_state, workpiece_length, workpiece_width = get_model_state(workpiece_model_name)

    if workpiece_state is not None:
        
        # Set boundaries for hole positions relative to workpiece center
        hole_boundaries_x = [-workpiece_length / 4, workpiece_length / 4]
        hole_boundaries_y = [-workpiece_width / 4, workpiece_width / 4]

        # Set random position within the defined boundaries
        pose = Pose()
        pose.position.x = workpiece_state.pose.position.x + uniform(*hole_boundaries_x)
        pose.position.y = workpiece_state.pose.position.y + uniform(*hole_boundaries_y)
        pose.position.z = workpiece_state.pose.position.z  # Assuming holes are on the same plane

        # Set random orientation in workpiece coordinate system
        # Convert the orientation from the world coordinate system to the workpiece coordinate system
        quat_world = [workpiece_state.pose.orientation.x, 
                      workpiece_state.pose.orientation.y,
                      workpiece_state.pose.orientation.z,
                      workpiece_state.pose.orientation.w]
        _, _, yaw_world = euler_from_quaternion(quat_world)

        # Add a random yaw offset in workpiece coordinate system
        yaw_offset = uniform(-1.57, 1.57)  # Adjust the desired range for yaw offset
        quat_workpiece = quaternion_from_euler(0, 0, yaw_world + yaw_offset)
        pose.orientation.x = quat_workpiece[0]
        pose.orientation.y = quat_workpiece[1]
        pose.orientation.z = quat_workpiece[2]
        pose.orientation.w = quat_workpiece[3]

        # Set the new state
        hole_state = ModelState(hole_model_name, pose)
        set_state(hole_state)

def get_model_state(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    try:
        model_state = get_model_state_proxy(model_name, "world")
        if model_state is not None:
            # Extract workpiece dimensions if available
            workpiece_length = model_state.dimensions.x if hasattr(model_state, 'dimensions') else 1.0
            workpiece_width = model_state.dimensions.y if hasattr(model_state, 'dimensions') else 1.0
            return model_state, workpiece_length, workpiece_width
        else:
            return None, None, None
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to get model state for '{model_name}': {str(e)}")
        return None, None, None

if __name__ == '__main__':
    rospy.init_node('set_hole_position')
    
    time.sleep(0.1)  # Add a delay    
    set_hole_position()
    
    time.sleep(0.1)  # Add a delay
    rospy.spin()
