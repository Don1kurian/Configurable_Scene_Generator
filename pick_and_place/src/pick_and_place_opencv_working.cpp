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

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

//Functions for Moving and grasping with robot
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion initPose

void initPose(moveit::planning_interface::MoveGroupInterface& move_group)
{ 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup("panda_manipulator");
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion pickPose as Cartesian Motion

void pickPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction, float x, float y){
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.2);
  move_group_interface.setMaxAccelerationScalingFactor(0.2);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_pick = move_group_interface.getCurrentPose().pose;
  target_pose_pick.position.x = x;
  target_pose_pick.position.y = y;
  if (direction == "down"){
    target_pose_pick.position.z -= 0.28;//0.26
  }
  else if (direction == "up"){
    target_pose_pick.position.z = 0.4;
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
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_manipulator");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  cartesianPlan.trajectory_ = trajectory_msg;

  if (direction == "down"){
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm* local_time = std::localtime(&now_time);
  int month = local_time->tm_mon + 1;    // Month of the year (1 - 12)
  int day = local_time->tm_mday;         // Day of the month (1 - 31)
  int hour = local_time->tm_hour;
  int minute = local_time->tm_min;      // Minute of the hour (0 - 59)
  int seconds = local_time->tm_sec;      // Seconds (0 - 59)

  // Create the filename with the month, date, hour, and seconds
  std::stringstream ss;
  ss << "/home/USADR/ac139860/image_FI/src/data/pick_trajectory/pick-" << month << "-" << day
      << "_" << hour << ":" << minute << ":" << seconds << ".csv";

  std::ofstream csv_file(ss.str());
    if (!csv_file.is_open()) {
        ROS_ERROR_STREAM("Failed to open CSV file for writing: " << ss.str());
        return;
    }

    csv_file << "Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7\n";

    for (const auto& point : trajectory_msg.joint_trajectory.points) {
        for (size_t i = 0; i < point.positions.size(); ++i) {
            csv_file << point.positions[i];
            if (i < point.positions.size() - 1) {
                csv_file << ",";
            }
        }
        csv_file << "\n";
    }
  }
  move_group_interface.execute(cartesianPlan);  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion hoverPlacePose

void hoverPlacePose(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose pose_hover_place;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY(-tau/2, 0, 0);

  pose_hover_place.orientation = tf2::toMsg(orientation);
  
  pose_hover_place.position.x = x;
  pose_hover_place.position.y = y;
  pose_hover_place.position.z = 0.33365;
  move_group.setPoseTarget(pose_hover_place);

  move_group.move();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion PlacePose

void PlacePose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction,float x, float y)
{ 
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.2);
  move_group_interface.setMaxAccelerationScalingFactor(0.2);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_place = move_group_interface.getCurrentPose().pose;
  target_pose_place.position.x = x;
  target_pose_place.position.y = y;
  if (direction == "down"){
    target_pose_place.position.z -= 0.28; 
   
  }

  else if (direction == "up"){
    target_pose_place.position.z = 0.4;
  }
  waypoints.push_back(target_pose_place); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
   
  // Create robot trajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_manipulator");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);
  
  cartesianPlan.trajectory_ = trajectory_msg;

  if (direction == "down"){
  
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm* local_time = std::localtime(&now_time);
  int month = local_time->tm_mon + 1;    // Month of the year (1 - 12)
  int day = local_time->tm_mday;         // Day of the month (1 - 31)
  int hour = local_time->tm_hour;
  int minute = local_time->tm_min;      // Minute of the hour (0 - 59)
  int seconds = local_time->tm_sec;      // Seconds (0 - 59)

  // Create the filename with the month, date, hour, and seconds
  std::stringstream ss;
  ss << "/home/USADR/ac139860/image_FI/src/data/place_trajectory/place-" << month << "-" << day
      << "_" << hour << ":" << minute << ":" << seconds << ".csv";

  std::ofstream csv_file(ss.str());
    if (!csv_file.is_open()) {
        ROS_ERROR_STREAM("Failed to open CSV file for writing: " << ss.str());
        return;
    }

    csv_file << "Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7\n";

    for (const auto& point : trajectory_msg.joint_trajectory.points) {
        for (size_t i = 0; i < point.positions.size(); ++i) {
            csv_file << point.positions[i];
            if (i < point.positions.size() - 1) {
                csv_file << ",";
            }
        }
        csv_file << "\n";
    }
    csv_file.close();
  
  }

  move_group_interface.execute(cartesianPlan);  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// close Gripper for moveit msg grasp

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

  // Add both finger joints
  posture.header.stamp = ros::Time::now();
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  //set closed position
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.0144;
  posture.points[0].positions[1] = 0.0144;
  posture.points[0].effort.resize(2);
  posture.points[0].effort[0] = 0.50;
  posture.points[0].effort[1] = 0.50;
   
  
  posture.points[0].time_from_start = ros::Duration(1);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Little movement for grasping with grasp msg

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  //Create Vector for grasp approaches (only need 1)
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";

  // Setting grasp pose
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  grasps[0].grasp_pose = current_pose;
  grasps[0].grasp_pose.pose.position.z -= 0.0001;
  grasps[0].grasp_pose.header.frame_id = "panda_link0";

  // Direction is set as negative z axis, approach from above the object
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.0001;
  grasps[0].pre_grasp_approach.desired_distance = 0.0002;

  //Close gripper to grasp object
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("box1", grasps);
  
}

///////////////////////////////////////////////////////////////////////////////////////
// Plan and execute open hand

void openHand(moveit::planning_interface::MoveGroupInterface& move_group_interface_hand)
{ 
  // Open the gripper
  move_group_interface_hand.setJointValueTarget(move_group_interface_hand.getNamedTargetValues("open"));

  //Move the robot
  // ROS_WARN("START EXECUTION");
  move_group_interface_hand.move();

}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, float x, float y)
{

  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cylinder will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1;
  collision_objects[0].primitives[0].dimensions[1] = 1.8;
  collision_objects[0].primitives[0].dimensions[2] = 0;

  // Define the pose of the table. 
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.2;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.01;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;


  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the wall at the back of the robot.
  collision_objects[1].id = "wallback";
  collision_objects[1].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0;
  collision_objects[1].primitives[0].dimensions[1] = 1.8;
  collision_objects[1].primitives[0].dimensions[2] = 1.0;

  // Define the pose of the wall. 
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.3;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0.5;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;


  collision_objects[1].operation = collision_objects[1].ADD;

  // Add the second wall on the left hand side
  collision_objects[2].id = "wallright";
  collision_objects[2].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 1.2;
  collision_objects[2].primitives[0].dimensions[1] = 0;
  collision_objects[2].primitives[0].dimensions[2] = 1.0;

  // Define the pose of the wall on the right. 
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.2;
  collision_objects[2].primitive_poses[0].position.y = 0.9;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;


  collision_objects[2].operation = collision_objects[2].ADD;

    // Add the second wall on the left hand side
  collision_objects[3].id = "plate";
  collision_objects[3].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions. 
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(2);
  collision_objects[3].primitives[0].dimensions[0] = 0.03;
  collision_objects[3].primitives[0].dimensions[1] = 0.025;


  // Define the pose of the wall on the right. 
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = x;
  collision_objects[3].primitive_poses[0].position.y = y;
  collision_objects[3].primitive_poses[0].position.z = 0.01;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;


  collision_objects[3].operation = collision_objects[3].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
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
  // moveit::planning_interface::MoveGroupInterface group_arm("panda_manipulator");
  // moveit::planning_interface::MoveGroupInterface group_hand("panda_hand");
  moveit::planning_interface::MoveGroupInterface group_arm("panda_arm");
  
  // Set parameters for group like planner, speed, acceleration
  group_arm.setPlannerId("RRTConnect");
  group_arm.setMaxVelocityScalingFactor(0.2);
  group_arm.setMaxAccelerationScalingFactor(0.2);
  // group_arm.setPlannerId("RRTConnect");
  // group_arm.setMaxVelocityScalingFactor(0.2);
  // group_arm.setMaxAccelerationScalingFactor(0.2);
  //group_arm.setNumPlanningAttempts(2);
  
  std::ifstream file("/home/don/your_workspace/src/panda_gazebo/docs/hole_position.txt");
    
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open file");
        return 1;  // Exit with an error code
    }

    float hole1_x, hole1_y, hole1_z;
    float hole2_x, hole2_y, hole2_z;
    float hole3_x, hole3_y, hole3_z;

    file >> hole1_x >> hole1_y >> hole1_z;
    file >> hole2_x >> hole2_y >> hole2_z;
    file >> hole3_x >> hole3_y >> hole3_z;

    file.close();



  // call the service of obtaining the location msg
  // ros::ServiceClient box_and_target_position_srv_client = n.serviceClient<opencv_services::box_and_target_position>("box_and_target_position");
  // opencv_services::box_and_target_position srv;
  //   if(box_and_target_position_srv_client.call(srv)) {
  //   ROS_INFO_STREAM("3d target position camera frame: x " << srv.response.target_position.x << " y " << srv.response.target_position.y << " z " << srv.response.target_position.z);
  //   ROS_INFO_STREAM("3d box position camera frame: x " << srv.response.box_position.x << " y " << srv.response.box_position.y << " z " << srv.response.box_position.z);
  // } else {
  //   ROS_INFO_STREAM("Failed to call box and target position service");
  // }
  // // create the pose of box
  // geometry_msgs::Pose box_pose;
  //   if(srv.response.box_position.x < 0.2){
  //   box_pose.orientation.w = 1.0;
  //   box_pose.position.x = srv.response.box_position.x + 0.25;
  //   box_pose.position.y = srv.response.box_position.y;
  //   box_pose.position.z = srv.response.box_position.z;
  // } else{
  //   box_pose.orientation.w = 1.0;
  //   box_pose.position.x = srv.response.box_position.x;
  //   box_pose.position.y = srv.response.box_position.y;
  //   box_pose.position.z = srv.response.box_position.z;
  // }
  // ros::Duration(0.1).sleep();
  // // create the pose of target
  // geometry_msgs::Pose target_pose;
  // // target_pose.orientation = current_pose.pose.orientation;
  // target_pose.position.x = srv.response.target_position.x;
  // target_pose.position.y = srv.response.target_position.y;
  // target_pose.position.z = srv.response.target_position.z + 0.34;
   // Add Objects to the environment
  // addCollisionObjects(planning_scene_interface, target_pose.position.x, target_pose.position.y);
  //Create Box
  // shape_msgs::SolidPrimitive primitive;
  // moveit_msgs::CollisionObject object_to_attach;
  // object_to_attach.id = "box1";
  // shape_msgs::SolidPrimitive box_primitive;
  //  box_primitive.type = primitive.BOX;
  // box_primitive.dimensions.resize(3);
  // box_primitive.dimensions[primitive.BOX_X] = 0.03;
  // box_primitive.dimensions[primitive.BOX_Y] = 0.03;
  // box_primitive.dimensions[primitive.BOX_Z] = 0.03;
  // // define the frame/pose for this box
  // object_to_attach.header.frame_id = "panda_link0";
  // geometry_msgs::Pose grab_pose;
  // grab_pose.orientation.w = 1.0;
  // grab_pose.position.x = box_pose.position.x;
  // grab_pose.position.y = box_pose.position.y;
  // grab_pose.position.z = 0.01;
  // First, we add the object to the world (without using a vector)
  // object_to_attach.primitives.push_back(box_primitive);
  // object_to_attach.primitive_poses.push_back(grab_pose);
  // object_to_attach.operation = object_to_attach.ADD;
  // planning_scene_interface.applyCollisionObject(object_to_attach);
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  //  openHand(group_hand);

  hoverPose(group_arm, hole1_x, hole1_y, hole1_z);

  // pickPose(group_arm  , "down", 0.4, 0.1);

  // pick(group_arm);

  // group_hand.attachObject(object_to_attach.id);

  pickPose(group_arm , "up", 0.4, -0.1);

  hoverPlacePose(group_arm, 0.4, 0.1);

  PlacePose(group_arm , "down", 0.4, 0.1);
  
  ros::WallDuration(1.0).sleep();
  // openHand(group_hand);
  // group_hand.detachObject(object_to_attach.id);
  
  // PlacePose(group_arm , "up", 0.4, 0.1);



  // initPose(group_arm);

  ros::WallDuration(2.0).sleep();
  ROS_WARN("round end");

  
  
  ros::shutdown();
  return 0;
}