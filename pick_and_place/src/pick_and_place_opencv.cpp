#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
// #include "opencv_services/box_and_target_position.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion hoverPose
void hoverPose(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y, float z)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose_hover;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY(-tau/2, 0, 0);

  target_pose_hover.orientation = tf2::toMsg(orientation);
  
  target_pose_hover.position.x = x;
  target_pose_hover.position.y = y;
  target_pose_hover.position.z = z;
  move_group.setPoseTarget(target_pose_hover);

  move_group.move();
  
}

void initPose(moveit::planning_interface::MoveGroupInterface& move_group)
{ 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup("panda_arm");
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0;
  joint_group_positions[1] = -tau / 8;  // -1/8 turn in radians
  joint_group_positions[2] = 0;
  joint_group_positions[3] = -3 * tau / 8;  // -8/8 turn in radians
  joint_group_positions[4] = 0;
  joint_group_positions[5] = tau / 4 + 0.03;  // 1/4 turn in radians
  joint_group_positions[6] = tau / 8;  // 1/8 turn in radians
  move_group.setJointValueTarget(joint_group_positions);
  //
  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group.setMaxVelocityScalingFactor(0.2); // default 0.05
  move_group.setMaxAccelerationScalingFactor(0.2); // default 0.05
  //
  move_group.move();
}

void pickPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction, float x, float y, float z){
  
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setMaxVelocityScalingFactor(0.01);
  move_group_interface.setMaxAccelerationScalingFactor(0.01);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_pick = move_group_interface.getCurrentPose().pose;
  target_pose_pick.position.x = x;
  target_pose_pick.position.y = y;
  if (direction == "down"){
    target_pose_pick.position.z = z-0.059;
  }
  else if (direction == "up"){
    target_pose_pick.position.z = z+0.045;
  }
  
  waypoints.push_back(target_pose_pick);

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
  
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  // Modify trajectory for adjusting speed
  
  // Create robot trajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_arm");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);
  cartesianPlan.trajectory_ = trajectory_msg;
  move_group_interface.execute(cartesianPlan);  

}

// void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, float x, float y)
// {
//   // Create vector to hold 3 collision objects.
//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.resize(4);
//   // Add the first table where the cylinder will originally be kept.
//   collision_objects[0].id = "table1";
//   collision_objects[0].header.frame_id = "panda_link0";
//   // Define the primitive and its dimensions. 
//   collision_objects[0].primitives.resize(1);
//   collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
//   collision_objects[0].primitives[0].dimensions.resize(3);
//   collision_objects[0].primitives[0].dimensions[0] = 1;
//   collision_objects[0].primitives[0].dimensions[1] = 1.8;
//   collision_objects[0].primitives[0].dimensions[2] = 0;
//   // Define the pose of the table. 
//   collision_objects[0].primitive_poses.resize(1);
//   collision_objects[0].primitive_poses[0].position.x = 0.2;
//   collision_objects[0].primitive_poses[0].position.y = 0;
//   collision_objects[0].primitive_poses[0].position.z = -0.01;
//   collision_objects[0].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[0].operation = collision_objects[0].ADD;
//   // Add the wall at the back of the robot.
//   collision_objects[1].id = "wallback";
//   collision_objects[1].header.frame_id = "panda_link0";
//   // Define the primitive and its dimensions. 
//   collision_objects[1].primitives.resize(1);
//   collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[1].primitives[0].dimensions.resize(3);
//   collision_objects[1].primitives[0].dimensions[0] = 0;
//   collision_objects[1].primitives[0].dimensions[1] = 1.8;
//   collision_objects[1].primitives[0].dimensions[2] = 1.0;
//   // Define the pose of the wall. 
//   collision_objects[1].primitive_poses.resize(1);
//   collision_objects[1].primitive_poses[0].position.x = -0.3;
//   collision_objects[1].primitive_poses[0].position.y = 0;
//   collision_objects[1].primitive_poses[0].position.z = 0.5;
//   collision_objects[1].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[1].operation = collision_objects[1].ADD;
//   // Add the second wall on the left hand side
//   collision_objects[2].id = "wallright";
//   collision_objects[2].header.frame_id = "panda_link0";
//   // Define the primitive and its dimensions. 
//   collision_objects[2].primitives.resize(1);
//   collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[2].primitives[0].dimensions.resize(3);
//   collision_objects[2].primitives[0].dimensions[0] = 1.2;
//   collision_objects[2].primitives[0].dimensions[1] = 0;
//   collision_objects[2].primitives[0].dimensions[2] = 1.0;
//   // Define the pose of the wall on the right. 
//   collision_objects[2].primitive_poses.resize(1);
//   collision_objects[2].primitive_poses[0].position.x = 0.2;
//   collision_objects[2].primitive_poses[0].position.y = 0.9;
//   collision_objects[2].primitive_poses[0].position.z = 0.5;
//   collision_objects[2].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[2].operation = collision_objects[2].ADD;
//     // Add the second wall on the left hand side
//   collision_objects[3].id = "plate";
//   collision_objects[3].header.frame_id = "panda_link0";
//   // Define the primitive and its dimensions. 
//   collision_objects[3].primitives.resize(1);
//   collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
//   collision_objects[3].primitives[0].dimensions.resize(2);
//   collision_objects[3].primitives[0].dimensions[0] = 0.03;
//   collision_objects[3].primitives[0].dimensions[1] = 0.025;
//   // Define the pose of the wall on the right. 
//   collision_objects[3].primitive_poses.resize(1);
//   collision_objects[3].primitive_poses[0].position.x = x;
//   collision_objects[3].primitive_poses[0].position.y = y;
//   collision_objects[3].primitive_poses[0].position.z = 0.01;
//   collision_objects[3].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[3].operation = collision_objects[3].ADD;
//   planning_scene_interface.applyCollisionObjects(collision_objects);
// }


/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  float hole1_x, hole1_y, hole1_z;
  float hole2_x, hole2_y, hole2_z;
  float hole3_x, hole3_y, hole3_z;

  ros::init(argc, argv, "own_pick_place_V4");
  ros::NodeHandle nh;
  ros::NodeHandle n;
  //Get information about robot state
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::WallDuration(1.0).sleep();
  
  // use for planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //planning interface
  moveit::planning_interface::MoveGroupInterface group_arm("panda_arm");
  
  // Set parameters for group like planner, speed, acceleration
  group_arm.setPlannerId("RRTConnect");
  group_arm.setMaxVelocityScalingFactor(0.2);
  group_arm.setMaxAccelerationScalingFactor(0.2);
  group_arm.setPlanningTime(5.0);

  gazebo_msgs::GetModelState get_model_state;

  std::string model_name_1="hole_1";
  get_model_state.request.model_name = model_name_1;
  // Wait for the service to be available
  ros::service::waitForService("/gazebo/get_model_state", -1);
  // Call the service
  if (ros::service::call("/gazebo/get_model_state", get_model_state))
    {    // Return the pose values as a tuple
      hole1_x=get_model_state.response.pose.position.x;
      hole1_y=get_model_state.response.pose.position.y;
      hole1_z=get_model_state.response.pose.position.z;
      // Display the loaded values
      std::cout << model_name_1 << " position: "
                << get_model_state.response.pose.position.x << ", "
                << get_model_state.response.pose.position.y << ", "
                << get_model_state.response.pose.position.z << std::endl;}
  else
    {std::cerr << "Failed to call get_model_state service" << std::endl;
      return -1;  }

  std::string model_name_2="hole_2";
  get_model_state.request.model_name = model_name_2;
  // Wait for the service to be available
  ros::service::waitForService("/gazebo/get_model_state", -1);
  // Call the service
  if (ros::service::call("/gazebo/get_model_state", get_model_state))
    {    // Return the pose values as a tuple
      hole2_x=get_model_state.response.pose.position.x;
      hole2_y=get_model_state.response.pose.position.y;
      hole2_z=get_model_state.response.pose.position.z;
      // Display the loaded values
      std::cout << model_name_2 << " position: "
                << get_model_state.response.pose.position.x << ", "
                << get_model_state.response.pose.position.y << ", "
                << get_model_state.response.pose.position.z << std::endl;}
  else
    {std::cerr << "Failed to call get_model_state service" << std::endl;
      return -1;  }    

  std::string model_name_3="hole_3";
  get_model_state.request.model_name = model_name_3;
  // Wait for the service to be available
  ros::service::waitForService("/gazebo/get_model_state", -1);
  // Call the service
  if (ros::service::call("/gazebo/get_model_state", get_model_state))
    {    // Return the pose values as a tuple
      hole3_x=get_model_state.response.pose.position.x;
      hole3_y=get_model_state.response.pose.position.y;
      hole3_z=get_model_state.response.pose.position.z;
      // Display the loaded values
      std::cout << model_name_3 << " position: "
                << get_model_state.response.pose.position.x << ", "
                << get_model_state.response.pose.position.y << ", "
                << get_model_state.response.pose.position.z << std::endl;}
  else
    {std::cerr << "Failed to call get_model_state service" << std::endl;
      return -1;  }
  ros::WallDuration(1.0).sleep();
    
  // hoverPose(group_arm, 0.5,0.1,0.9);
  // pickPose(group_arm, "down",0.5,0.1,0.9);  
  hoverPose(group_arm, hole1_x, hole1_y, hole1_z+0.24 );
  pickPose(group_arm, "down",hole1_x, hole1_y, hole1_z+0.24);
  ros::Duration(2.0).sleep();  // Delay for 2 seconds
  pickPose(group_arm, "up",hole1_x, hole1_y, hole1_z+0.24);
  initPose(group_arm);
  ROS_WARN("round1 end");
  hoverPose(group_arm, hole2_x, hole2_y, hole2_z+0.24);
  pickPose(group_arm, "down",hole2_x, hole2_y, hole2_z+0.24);
  ros::Duration(3.0).sleep();  // Delay for 2 seconds
  pickPose(group_arm, "up",hole2_x, hole2_y, hole2_z+0.24);
  initPose(group_arm);
  ROS_WARN("round2 end");
  hoverPose(group_arm, hole3_x, hole3_y, hole3_z+0.24);
  pickPose(group_arm, "down",hole3_x, hole3_y, hole3_z+0.24);
  ros::Duration(3.0).sleep();  // Delay for 2 seconds
  pickPose(group_arm, "up",hole3_x, hole3_y, hole3_z+0.24);
  initPose(group_arm);
  ROS_WARN("round3 end");
  ros::WallDuration(3.0).sleep();
  ROS_WARN("round end");
  
  ros::shutdown();
  return 0;
}