#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<vector>
#include<kinova_moveit_bridge/kinova_moveit_bridge_node.hpp>

#include<thread>
#include<iostream>


using namespace std::placeholders;

KinovaMoveitBridge::KinovaMoveitBridge(const rclcpp::NodeOptions &options)
                    :rclcpp::Node("kinova_moveit_bridge",options)
                    {
                        // Action server intitialization with 3 callbacks
                        action_server_=rclcpp_action::create_server<FollowJointTrajectory>(
                            this,
                            "follow_joint_trajectory",
                            std::bind(&KinovaMoveitBridge::handle_goal,this,_1,_2),
                            std::bind(&KinovaMoveitBridge::handle_cancel,this,_1),
                            std::bind(&KinovaMoveitBridge::handle_accepted,this,_1)
                        );
                    }

rclcpp_action::GoalResponse KinovaMoveitBridge::handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal){
             
                (void)uuid;
                
                if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty())
                 {
                    return rclcpp_action::GoalResponse::REJECT;
                 }

                 return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;


            }

rclcpp_action::CancelResponse KinovaMoveitBridge::handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle){
                
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

void KinovaMoveitBridge::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
    std::thread{std::bind(&KinovaMoveitBridge::execute,this,goal_handle)}.detach();  
} 


void KinovaMoveitBridge::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }

    std::vector<TrajectoryPoint> traj = convertTrajectory(goal->trajectory);

    RCLCPP_INFO(get_logger(), " Received trajectory with %zu waypoints", traj.size());

    // TODO: call kinova_.executeTrajectory(traj) when KinovaInterface is wired in

    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Trajectory execution succeeded");
}

std::vector<KinovaMoveitBridge::TrajectoryPoint> KinovaMoveitBridge::convertTrajectory(const JointTrajectory& traj)
            {
            
                std::vector<TrajectoryPoint> result;
                for (size_t i = 0; i < traj.points.size(); i++) {
                    TrajectoryPoint pt;
                    pt.joint_angles = traj.points[i].positions;
                    pt.time_from_start = traj.points[i].time_from_start.sec +
                                        traj.points[i].time_from_start.nanosec * 1e-9;
                    result.push_back(pt);
                }
                return result;

            }
        
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinovaMoveitBridge>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}       

                             




