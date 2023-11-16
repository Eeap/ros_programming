// student ID: 2018102173
// name: sumin kim

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <iostream>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
geometry_msgs::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw)
{

  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll,pitch, yaw);
  target_pose.orientation= tf2::toMsg(orientation);
  target_pose.position.x=x;
  target_pose.position.y=y;
  target_pose.position.z=z;

  return target_pose;
}

void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
geometry_msgs::Pose &target_pose) {

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  move_group_interface.execute(my_plan);

}

void gripper_move(moveit::planning_interface::MoveGroupInterface &move_group_interface,double value)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  std::vector<double> joint_group_positions;

  joint_group_positions=move_group_interface.getCurrentJointValues();
  joint_group_positions[0]=0;
  joint_group_positions[1]=0;
  joint_group_positions[2]=value;
  joint_group_positions[3]=0;
  move_group_interface.setJointValueTarget(joint_group_positions);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan);
}

void p_n_p(moveit::planning_interface::MoveGroupInterface& arm_interface, moveit::planning_interface::MoveGroupInterface& gripper_interface)
{
    std::vector<std::vector<double>> block_pos_list = {
        {0.2,0.1,0.06}, 
        {0.2,0.0,0.06}, 
        {0.2,-0.1,0.06}, 
        {0.3,0.1,0.08}, 
        {0.3,0.0,0.06}, 
        {0.3,-0.1,0.07}, 
        {0.4,0.1,0.06}, 
        {0.4,0.0,0.09}
    };
    std::vector<std::vector<double>> target_pos_list = {
        {-0.15,0.35,0.06}, 
        {-0.05,0.35,0.06}, 
        {0.15,0.25,0.06}, 
        {0.05,0.35,0.08}, 
        {-0.05,0.25,0.06}, 
        {0.15,0.35,0.07}, 
        {0.05,0.25,0.06}, 
        {-0.15,0.25,0.09}, 
    };
    for (int i=0;i<8;i++) {
        //grab
        geometry_msgs::Pose block_pose;
        block_pose=list_to_pose(block_pos_list[i][0], block_pos_list[i][1], block_pos_list[i][2]+0.05, -M_PI,0,0);
        ROS_INFO("block pos move");
        go_to_pose_goal(arm_interface,block_pose);
        ros::WallDuration(2.0).sleep();

        ROS_INFO("gripper move");
        gripper_move(gripper_interface,0.5);
        ros::WallDuration(2.0).sleep();
        
        block_pose=list_to_pose(block_pos_list[i][0], block_pos_list[i][1], block_pos_list[i][2]-0.01, -M_PI,0,0);
        ROS_INFO("block pos down move");
        go_to_pose_goal(arm_interface,block_pose);
        ros::WallDuration(2.0).sleep();

        ROS_INFO("gripper move");
        if (i == 6) {
          gripper_move(gripper_interface,0.275);
        }
        else {
          gripper_move(gripper_interface,0.377);
        }
        ros::WallDuration(1.0).sleep();

        block_pose=list_to_pose(block_pos_list[i][0], block_pos_list[i][1], block_pos_list[i][2]*3, -M_PI,0,0);
        ROS_INFO("block pos up move");
        go_to_pose_goal(arm_interface,block_pose);
        ros::WallDuration(2.0).sleep();

        //out
        geometry_msgs::Pose target_pose;
        target_pose=list_to_pose(target_pos_list[i][0], target_pos_list[i][1], target_pos_list[i][2]*3,-M_PI,0,0);
        ROS_INFO("target move");
        go_to_pose_goal(arm_interface,target_pose);
        ros::WallDuration(2.0).sleep();
        
        target_pose=list_to_pose(target_pos_list[i][0], target_pos_list[i][1], target_pos_list[i][2]+0.05,-M_PI,0,0);
        ROS_INFO("target down move");
        go_to_pose_goal(arm_interface,target_pose);
        ros::WallDuration(.0).sleep();

        ROS_INFO("gripper move");
        gripper_move(gripper_interface,0.5);
        ros::WallDuration(2.0).sleep();

        target_pose=list_to_pose(target_pos_list[i][0], target_pos_list[i][1], target_pos_list[i][2]*3,-M_PI,0,0);
        ROS_INFO("target up move");
        go_to_pose_goal(arm_interface,target_pose);
        ros::WallDuration(2.0).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rp_midterm");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    arm.setPlanningTime(200.0);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    p_n_p(arm, gripper);    // execute p_n_p

    return 0;
}
