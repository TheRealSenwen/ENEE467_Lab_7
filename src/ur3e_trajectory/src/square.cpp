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
    pose_target.position.x = -0.45;
    pose_target.position.y = 0.0;
    pose_target.position.z = 0.77+0.128;
    pose_target.orientation.x = 1.0;
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
    std::cout << start_pose.position.x << "," << start_pose.position.y << "," << start_pose.position.z << std::endl;
    double z0 = 0.77+0.128;

    // Generate square path (horizontal)
    double horizontal_square_path_x[] = {-0.45, 0.0, 0.45, 0.0, -0.45};
    double horizontal_square_path_y[] = {0.0, 0.45, 0.0, -0.45, 0};
    double horizontal_square_path_z[] = {z0, z0, z0, z0, z0};

    // Generate square path (vertical)
    double vertical_square_path_x[] = {-0.3, -0.3, -0.3, -0.3, -0.3};
    double vertical_square_path_y[] = {0.0, 0.1, 0.0, -0.1, 0.0};
    double vertical_square_path_z[] = {z0, z0+0.1, z0+0.2, z0+0.1, z0};

    // Generate path for largest possible square
    double r = 0.45;
    double tilt = M_PI/18;

    double largest_path_x[] = {-r, 0.0, r, 0.0, -r};
    double largest_path_y[] = {0.0, r*cos(tilt), 0.0, -r*cos(tilt), 0.0};
    double largest_path_z[] = {z0, z0+r*sin(tilt), z0, z0-r*sin(tilt), z0};

    // Define waypoints for cartesian path
    geometry_msgs::Pose end_pose = start_pose;
    std::vector<geometry_msgs::Pose> waypoints;


    int path_sel = 1; //1: Largest, 2: Horizontal, 3: Vertical

    if(path_sel == 1){
        double* path_x = largest_path_x;
        double* path_y = largest_path_y;
        double* path_z = largest_path_z;

        for(int i = 0; i < 5; i++){
            end_pose.position.x = path_x[i];
            end_pose.position.y = path_y[i];
            end_pose.position.z = path_z[i];
            waypoints.push_back(end_pose);
        }
    }

    if(path_sel == 2){
        double* path_x = horizontal_square_path_x;
        double* path_y = horizontal_square_path_y;
        double* path_z = horizontal_square_path_z;

        for(int i = 0; i < 5; i++){
            end_pose.position.x = path_x[i];
            end_pose.position.y = path_y[i];
            end_pose.position.z = path_z[i];
            waypoints.push_back(end_pose);
        }
    }

    if(path_sel == 3){
        double* path_x = vertical_square_path_x;
        double* path_y = vertical_square_path_y;
        double* path_z = vertical_square_path_z;

        for(int i = 0; i < 5; i++){
            end_pose.position.x = path_x[i];
            end_pose.position.y = path_y[i];
            end_pose.position.z = path_z[i];
            waypoints.push_back(end_pose);
        }
    }

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
}