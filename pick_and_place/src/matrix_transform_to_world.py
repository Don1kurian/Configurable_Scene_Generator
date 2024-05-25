#!/usr/bin/env python3

import numpy as np

def transform_from_local_to_world(position_in_local_coordinate_system, origin_of_local_cordinate_system):
    x,y,z,roll,pitch,yaw = origin_of_local_cordinate_system

    translation_matrix = np.array([[1, 0, 0, x],
                                   [0, 1, 0, y],
                                   [0, 0, 1, z],
                                   [0, 0, 0, 1]])
    
    rotation_matrix_roll = np.array([[1, 0, 0, 0],
                                     [0, np.cos(roll), -np.sin(roll), 0],
                                     [0, np.sin(roll), np.cos(roll), 0],
                                     [0, 0, 0, 1]])

    rotation_matrix_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch), 0],
                                      [0, 1, 0, 0],
                                      [-np.sin(pitch), 0, np.cos(pitch), 0],
                                      [0, 0, 0, 1]])

    rotation_matrix_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0, 0],
                                    [np.sin(yaw), np.cos(yaw), 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
  
    rotation_matrix=np.dot(rotation_matrix_yaw, np.dot(rotation_matrix_pitch, rotation_matrix_roll))
    

    # Apply Inverse Transformations
    world_position = np.dot(translation_matrix, np.dot(rotation_matrix, np.append(position_in_local_coordinate_system, 1)))
    return world_position[:3]

