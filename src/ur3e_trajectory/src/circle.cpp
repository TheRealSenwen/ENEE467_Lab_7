#include "../include/circle.hpp"
#include <cmath>

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
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

    //Write your code for following the circle trajectory here.
    
    std::string reference_frame = "world";

    // Create instance of joint target plan
    
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = -M_PI/18.0;
    joint_targets["shoulder_lift_joint"] = -M_PI/2.0-M_PI/18.0;
    joint_targets["shoulder_pan_joint"] = 0.0;
    joint_targets["wrist_1_joint"] = -M_PI/2.0-M_PI/18.0;
    joint_targets["wrist_2_joint"] = 0.0;
    joint_targets["wrist_3_joint"] = 0.0;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    // Create instance of pose target plan
    
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target;
    pose_target.position.x = -0.3;
    pose_target.position.y = 0.0;
    pose_target.position.z = 0.77+0.128+0.15;
    pose_target.orientation.x = 0.0;
    pose_target.orientation.y = 1.0;
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
    geometry_msgs::Pose end_pose = start_pose;

    // Define waypoints for cartesian path
    std::vector<geometry_msgs::Pose> waypoints;

    double radius = 0.1;
    double x_origin = pose_target.position.x;
    double y_origin = pose_target.position.y;
    double z_origin = pose_target.position.z;
    double x_coord, y_coord, z_coord;

    int number_of_points = 20;
    int plane = 1; // 1: XY, 2: XZ, 3: YZ

    for(int i = 0; i < number_of_points; i++){
        if(plane == 1){
            x_coord = x_origin + radius * cos(i*2*M_PI/number_of_points);
            y_coord = y_origin + radius * sin(i*2*M_PI/number_of_points);

            end_pose.position.x = x_coord;
            end_pose.position.y = y_coord;
            end_pose.position.z += 0.0;

            waypoints.push_back(end_pose);
        }

        if(plane == 2){
            x_coord = x_origin + radius * cos(i*2*M_PI/number_of_points);
            z_coord = z_origin + radius * sin(i*2*M_PI/number_of_points);
            std::cout << x_coord << "," << z_coord << std::endl;

            end_pose.position.x = x_coord;
            end_pose.position.y += 0.0;
            end_pose.position.z = z_coord;

            waypoints.push_back(end_pose);
        }

        if(plane == 3){
            y_coord = y_origin + radius * cos(i*2*M_PI/number_of_points);
            z_coord = z_origin + radius * sin(i*2*M_PI/number_of_points);

            end_pose.position.x += 0.0;
            end_pose.position.y = y_coord;
            end_pose.position.z = z_coord;

            waypoints.push_back(end_pose);
        }
        
    }

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
    
}