#!/usr/bin/env python3
import math
import numpy as np
import rospy
import tf
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped

def dh_params(joint_variable):

    joint_var = joint_variable
    M_PI = math.pi
    # Create DH parameters (data given by maker franka-emika)
    dh =  np.array([[ 0,        0,        0.333,   joint_var[0]],
                    [-M_PI/2,   0,        0,       joint_var[1]],
                    [ M_PI/2,   0,        0.316,   joint_var[2]],
                    [ M_PI/2,   0.0825,   0,       joint_var[3]],
                    [-M_PI/2,  -0.0825,   0.384,   joint_var[4]],
                    [ M_PI/2,   0,        0,       joint_var[5]],
                    [ M_PI/2,   0.088,    0.107,   joint_var[6]]])
    rospy.loginfo("DH matrix:\n%s", dh)
    return dh
    
def TF_matrix(i,dh):
    # Define Transformation matrix based on DH params
    alpha = dh[i][0]
    a = dh[i][1]
    d = dh[i][2]
    q = dh[i][1]  
    TF= np.array([[math.cos(q),     -math.sin(q)*math.cos(alpha),  math.sin(q)*math.sin(alpha),    a*math.cos(q)],
                  [math.sin(q),     math.cos(q)*math.cos(alpha),   -math.cos(q)*math.sin(alpha),     a*math.sin(q)],
                  [0,               math.sin(alpha),                math.cos(alpha),                 d],
                  [0,                0,                                0,                             1]])
    # TF = np.array([[math.cos(q),                  -math.sin(q),                      0,                     a                  ],
    #                [math.sin(q)*math.cos(alpha),   math.cos(q)*math.cos(alpha),      -math.sin(alpha),      -math.sin(alpha)*d ],
    #                [math.sin(q)*math.sin(alpha),   math.cos(q)*math.sin(alpha),      math.cos(alpha),       math.cos(alpha)*d  ],
    #                [0,                             0,                                0,                     1                  ]])
    return TF

    
def calculate_end_effector_pose(joint_positions):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    source_frame = 'panda_link0'
    dh_parameters = dh_params(joint_positions)
    T_01 = TF_matrix(0,dh_parameters)
    T_12 = TF_matrix(1,dh_parameters)
    T_23 = TF_matrix(2,dh_parameters)
    T_34 = TF_matrix(3,dh_parameters)
    T_45 = TF_matrix(4,dh_parameters)
    T_56 = TF_matrix(5,dh_parameters)
    T_67 = TF_matrix(6,dh_parameters)

    T_07 = T_01*T_12*T_23*T_34*T_45*T_56*T_67 
    print("T_07\n",T_07)
    # translations = T_07[:3,3]
    quaternions = tf.transformations.quaternion_from_matrix(T_07)
    translations = tf.transformations.translation_from_matrix(T_07)
    print("End Effector Position (X, Y, Z):", translations[0],translations[1],translations[2])
    # Writing data to the Pose message for publishing
    panda_pose = Pose()
    panda_pose.position.x = translations[0]
    panda_pose.position.y = translations[1]
    panda_pose.position.z = translations[2]
    panda_pose.orientation.x = quaternions[0]
    panda_pose.orientation.y = quaternions[1]
    panda_pose.orientation.z = quaternions[2]
    panda_pose.orientation.w = quaternions[3]
    print(panda_pose.position.x,panda_pose.position.y,panda_pose.position.z)
    
    point = PoseStamped()
    point.header.frame_id = source_frame
    point.header.stamp = rospy.Time.now()
    point.pose.position.x = panda_pose.position.x
    point.pose.position.y = panda_pose.position.y
    point.pose.position.z = panda_pose.position.z
    point.pose.orientation.w = 1.0
    try:
        converted_point = tfBuffer.transform(point, source_frame, timeout=rospy.Duration(10.0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform lookup failed!")
    print("End Effector Position (X, Y, Z):", converted_point.pose.position.x,converted_point.pose.position.y,converted_point.pose.position.z)   
    return converted_point.pose  

def joint_state_callback(joint_state_msg):
    joint_positions = joint_state_msg.position
    rospy.loginfo("Received joint positions: %s", joint_positions)
    ee_pose = calculate_end_effector_pose(joint_positions)
    # rospy.loginfo("End-effector pose in source frame:")
    # rospy.loginfo(ee_pose)

if __name__ == '__main__':
    rospy.init_node('end_effector_localization')
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.spin()        
    
