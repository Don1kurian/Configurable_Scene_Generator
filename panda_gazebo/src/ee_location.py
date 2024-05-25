#!/usr/bin/env python3
import math
import time
import numpy as np
import rospy
import tf
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose ,Point, Pose, PoseStamped
from get_model_info import get_model_state
from std_msgs.msg import Float32

def calculate_distance(point1, point2):
    distance= math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
    return distance

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
                    [ M_PI/2,   0.088,    0.107,   joint_var[6]],
                    [ 0,        0,        0.18,    0           ]])
    return dh
    
def TF_matrix(i,dh):
    # Define Transformation matrix based on DH params
    alpha = dh[i][0]
    a = dh[i][1]
    d = dh[i][2]
    q = dh[i][3]  
    TF = np.array([[math.cos(q),                  -math.sin(q),                      0,                     a                  ],
                   [math.sin(q)*math.cos(alpha),   math.cos(q)*math.cos(alpha),      -math.sin(alpha),      -math.sin(alpha)*d ],
                   [math.sin(q)*math.sin(alpha),   math.cos(q)*math.sin(alpha),      math.cos(alpha),       math.cos(alpha)*d  ],
                   [0,                             0,                                0,                     1                  ]])
    return TF


def joint_state_callback(joint_state_msg):
    global last_transform_time
    current_time = rospy.Time.now()
    if current_time - last_transform_time < rospy.Duration(0.1):  # Limit to transform calculation every 0.1 second
        return
    last_transform_time = current_time

    joint_positions = joint_state_msg.position
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
    T_7ee = TF_matrix(7,dh_parameters)

    T_07 = T_01@T_12@T_23@T_34@T_45@T_56@T_67@T_7ee

    # quaternions = tf.transformations.quaternion_from_matrix(T_07)
    translations = tf.transformations.translation_from_matrix(T_07)
    
    panda_pose = Pose()                                       # Writing data to the Pose message for publishing
    panda_pose.position.x = translations[0]
    panda_pose.position.y = translations[1]
    panda_pose.position.z = translations[2]
        
    point = PoseStamped()
    point.header.frame_id = source_frame
    point.header.stamp = rospy.Time.now()
    point.pose=panda_pose
    print("End Effector Position_world (X, Y, Z):", point.pose.position)
        
    try:
        # converted_point = tfBuffer.transform(point, "panda_link0", timeout=rospy.Duration(10.0))
        
        hand_model_pose_position=get_model_state("hand_1")
        hand_model_position=(hand_model_pose_position.position.x,hand_model_pose_position.position.y,hand_model_pose_position.position.z) 
        end_effector_position = (point.pose.position.x, point.pose.position.y, point.pose.position.z)
        distance = calculate_distance(end_effector_position, hand_model_position)
        distance_pub = rospy.Publisher('/distance_between_end_effector_and_hand', Float32, queue_size=10)
        distance_pub.publish(distance)
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform lookup failed!")



if __name__ == '__main__':
    rospy.init_node('end_effector_localization')
    
        
    last_transform_time = rospy.Time.now()
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.spin()
    
         
    
