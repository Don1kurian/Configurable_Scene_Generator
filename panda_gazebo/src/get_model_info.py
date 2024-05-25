#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import xml.etree.ElementTree as ET

def read_original_sdf(model_path):
    with open(model_path, 'r') as model_file:
        return model_file.read()

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


def get_model_dimensions(model_path):
    with open(model_path, 'r') as model_file:               # Read the original SDF file
        original_sdf =model_file.read()
    root = ET.fromstring(original_sdf)
    box_element = root.find('.//box')
    size_text = box_element.find('.//size').text
    length, width, height = map(float, size_text.split())
    workpiece_dimensions = (length, width, height)
    return workpiece_dimensions

def set_dimensions_in_sdf(model_path, model_dimension):
    x, y, z = model_dimension
    sdf_file_path = model_path  # Path to the SDF file of the model you want to resize  
    tree = ET.parse(sdf_file_path)  # Parse the SDF file
    root = tree.getroot()
    for box in root.iter('box'):  # Find and modify the size elements
        size_element = box.find('size')
        size_element.text = f'{x} {y} {z}'  # Update the size attributes with your desired dimensions
    modified_sdf_content = ET.tostring(root, encoding='unicode')
    tree.write(sdf_file_path)  # Write the modified SDF file
    with open(sdf_file_path, 'r') as file:
        modified_sdf_content = file.read()  # Read the modified SDF content
    return modified_sdf_content  # Return the modified SDF content


def get_model_pose(model_name):                    #get_model_pose provides x,y,z,roll,pitch,yaw
    _model_state= ModelState()
    _model_state.pose= Pose()
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state_call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        _model_state_ = get_model_state_call(model_name, "world")
        _model_state.pose = _model_state_.pose
        orientation_quaternion = (
                _model_state.pose.orientation.x,
                _model_state.pose.orientation.y,
                _model_state.pose.orientation.z,
                _model_state.pose.orientation.w
            )
        roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
        model_pose = (_model_state.pose.position.x,
                      _model_state.pose.position.y,
                      _model_state.pose.position.z,
                      roll,
                      pitch,
                      yaw                          )
        return model_pose
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to get model state for '{model_name}': {str(e)}")
        return None

def get_model_state(model_name):
    model_state= ModelState()
    model_state.pose= Pose()
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state_call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
        model_state = get_model_state_call(model_name, "world")
        return model_state.pose
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to get model state for '{model_name}': {str(e)}")
        return None

def write_pose_to_sdf_file(model_path, model_state_pose):
    _model_state= ModelState()
    _model_state.pose= Pose()
    _model_state.pose=model_state_pose
    orientation_quaternion = (
                _model_state.pose.orientation.x,
                _model_state.pose.orientation.y,
                _model_state.pose.orientation.z,
                _model_state.pose.orientation.w
            )
    roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
    model_pose = (_model_state.pose.position.x,
                  _model_state.pose.position.y,
                  _model_state.pose.position.z,
                  roll,
                  pitch,
                  yaw                          )
    x,y,z,roll,pitch,yaw = model_pose
    sdf_file_path= model_path
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()

    link = root.find(".//link[@name='link']")
    if link is not None:
        inertial = link.find("./inertial")
        if inertial is not None:
            pose = inertial.find("pose")
            if pose is not None:
                pose.text = f"{x} {y} {z} {roll} {pitch} {yaw}"
                modified_sdf_content = ET.tostring(root, encoding='unicode')
                tree.write(sdf_file_path)  # Write the modified SDF file
                with open(sdf_file_path, 'r') as file:
                    modified_sdf_content = file.read()
                return modified_sdf_content
            else:
                rospy.logerr("Failed to find <pose> element in the SDF.")
        else:
            rospy.logerr("Failed to find <inertial> element in the SDF.")
    else:
        rospy.logerr("Failed to find <link name='link'> element in the SDF.")


