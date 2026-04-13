#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<vector>

using FollowJointTrajectory= control_msgs::action::FollowJointTrajectory;
using JointTrajectory=trajectory_msgs::msg::JointTrajectory;
using GoalHandle=rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class KinovaMoveitBridge:public rclcpp::Node{
    public:
        explicit KinovaMoveitBridge(const rclcpp::NodeOptions& options);
  
    private:
        struct TrajectoryPoint
        {
            std::vector<double> joint_angles;
            double time_from_start;
        };

        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

        //3 Callbacks
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        
        rclcpp_action::CancelResponse handle_cancel(
            std::shared_ptr<GoalHandle> goal_handle);  //goal_nadle object

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);    
        
        void execute(const std::shared_ptr<GoalHandle> goal_hangle);
        
        std::vector<TrajectoryPoint> convertTrajectory(
            const JointTrajectory& traj);

        };