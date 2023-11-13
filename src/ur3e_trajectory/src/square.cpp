#include "../include/square.hpp"
#include <cmath>

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the square trajectory here.
    std::string reference_frame = "world";

    // Create instance of joint target plan
    /*
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 0.08*2;
    joint_targets["shoulder_lift_joint"] = -0.08;
    joint_targets["shoulder_pan_joint"] = 1.57;
    joint_targets["wrist_1_joint"] = -1.57-0.08;
    joint_targets["wrist_2_joint"] = 0.0;
    joint_targets["wrist_3_joint"] = 0.0;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }
    */

    // Create instance of pose target plan
    
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target;
    pose_target.position.x = 0.3;
    pose_target.position.y = 0.3;
    pose_target.position.z = 1.3;
    pose_target.orientation.x = 0.0;
    pose_target.orientation.y = 0.0;
    pose_target.orientation.z = 0.0;
    pose_target.orientation.w = 0.0;

    bool pose_plan_success;
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }
    

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;
    double z0 = start_pose.position.z;

    // Generate square path
    double square_path_x[] = {0.2, 0.2, 0.3, 0.3};
    double square_path_y[] = {0.3, 0.2, 0.2, 0.3};
    double square_path_z[] = {z0, z0, z0, z0};

    // Generate path for largest possible square
    double L = 0.4*cos(atan(2));
    double tilt = 0.02;

    double largest_path_x[] = {L, L, -L, -L};
    double largest_path_y[] = {0.0, 2*L*cos(tilt), 2*L*cos(tilt), 0.0};
    double largest_path_z[] = {z0, z0+L*sin(tilt), z0+L*sin(tilt), z0};

    // Define waypoints for cartesian path
    geometry_msgs::Pose end_pose = start_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    double* path_x = largest_path_x;
    double* path_y = largest_path_y;
    double* path_z = largest_path_z;

    for(int i = 0; i < 4; i++){
        end_pose.position.x = path_x[i];
        end_pose.position.y = path_y[i];
        end_pose.position.z = path_z[i];
        waypoints.push_back(end_pose);
    }

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
}