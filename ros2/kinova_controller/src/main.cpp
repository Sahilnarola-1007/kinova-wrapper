#include"kinova_controller/KinovaLifecycleController.hpp"
#include<rclcpp/rclcpp.hpp>
#include<rclcpp_lifecycle/lifecycle_node.hpp>
#include<iostream>
#include<memory>

int main(int argc,char **argv){

    rclcpp::init(argc,argv);
    auto node=std::make_shared<kinova_ros2::KinovaLifecycleController>
    (rclcpp::NodeOptions());
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    
    rclcpp::shutdown();

    return 0;
}