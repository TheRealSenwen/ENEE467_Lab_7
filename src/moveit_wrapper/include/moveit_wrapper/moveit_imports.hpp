#pragma once

// ROS
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

// RViz
#include <moveit_visual_tools/moveit_visual_tools.h>


// MoveIt Planning
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit_msgs/CollisionObject.h>

// Time-optimal trajectory planning
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// Robot State
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// CPP
#include <time.h>
#include <iostream>
#include <string>

// Custom
#include "../moveit_wrapper/moveit_planning_options.hpp"

//Gripper
#include <control_msgs/GripperCommandActionGoal.h>
