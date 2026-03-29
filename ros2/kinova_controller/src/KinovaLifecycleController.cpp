#include "kinova_controller/KinovaLifecycleController.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <iostream>
#include <thread>

#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace kinova_ros2 {

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

/**
 * @brief Convert ROS Pose message to Kinova wrapper Pose.
 *
 * Converts position directly and transforms orientation from quaternion
 * (ROS standard) to roll-pitch-yaw in degrees (required by Kinova API).
 *
 * @param ros_pose Input ROS pose
 * @return kinova_wrapper::Pose Converted pose for hardware interface
 */
kinova_wrapper::Pose toWrapperPose(const geometry_msgs::msg::Pose & ros_pose)
{
    kinova_wrapper::Pose p;
    p.x = ros_pose.position.x;
    p.y = ros_pose.position.y;
    p.z = ros_pose.position.z;

    tf2::Quaternion q;
    tf2::fromMsg(ros_pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert radians → degrees (Kinova API requirement)
    p.theta_x = roll * 180.0 / M_PI;
    p.theta_y = pitch * 180.0 / M_PI;
    p.theta_z = yaw * 180.0 / M_PI;

    return p;
}

/**
 * @brief Convert Kinova wrapper Pose to ROS Pose message.
 *
 * Converts position directly and transforms roll-pitch-yaw (degrees)
 * into quaternion representation used by ROS.
 *
 * @param wp Wrapper pose from Kinova interface
 * @return geometry_msgs::msg::Pose ROS-compatible pose
 */
geometry_msgs::msg::Pose toROSPose(const kinova_wrapper::Pose & wp)
{
    geometry_msgs::msg::Pose ros_pose;

    ros_pose.position.x = wp.x;
    ros_pose.position.y = wp.y;
    ros_pose.position.z = wp.z;

    tf2::Quaternion q;
    q.setRPY(
        wp.theta_x * M_PI / 180.0,
        wp.theta_y * M_PI / 180.0,
        wp.theta_z * M_PI / 180.0
    );

    ros_pose.orientation = tf2::toMsg(q);
    return ros_pose;
}

/**
 * @brief Compute Euclidean distance between two poses (position only).
 *
 * @param target_pose Desired pose
 * @param current_pose Current robot pose
 * @return double Distance in meters
 */
double compute_distance(
    const geometry_msgs::msg::Pose & target_pose,
    const geometry_msgs::msg::Pose & current_pose)
{
    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double dz = target_pose.position.z - current_pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

/**
 * @brief Constructor for KinovaLifecycleController.
 *
 * Declares configurable parameters. Resource allocation is deferred to
 * on_configure().
 */
KinovaLifecycleController::KinovaLifecycleController(
    const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("kinova_controller", options)
{
    declare_parameter("robot_ip", "192.168.1.10");
    declare_parameter("robot_port", 10000);
}

// -----------------------------------------------------------------------------
// Lifecycle: Configure
// -----------------------------------------------------------------------------

KinovaLifecycleController::CallbackReturn
KinovaLifecycleController::on_configure(const rclcpp_lifecycle::State& state)
{
    (void)state;

    // Step 1: Load parameters
    robot_ip_ = get_parameter("robot_ip").as_string();
    robot_port_ = static_cast<uint32_t>(get_parameter("robot_port").as_int());

    if (robot_ip_.empty()) {
        RCLCPP_ERROR(get_logger(), "Robot IP parameter is empty");
        return CallbackReturn::FAILURE;
    }

    // Step 2: Create interface (no connection yet)
    interface_ = std::make_unique<kinova_wrapper::KinovaInterface>();

    if (!interface_) {
        RCLCPP_ERROR(get_logger(), "Failed to create Kinova interface");
        return CallbackReturn::FAILURE;
    }

    // Step 3: Create publisher (best-effort QoS for sensor data)
    auto qos_sensor = rclcpp::QoS(1)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", qos_sensor);

    // Step 4: Create action server
    action_server_ = rclcpp_action::create_server<MoveToPose>(
        this,
        "/kinova/move_to_pose",
        std::bind(&KinovaLifecycleController::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&KinovaLifecycleController::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&KinovaLifecycleController::execute_goal, this,
                  std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
        "Configured successfully | IP: %s Port: %u",
        robot_ip_.c_str(), robot_port_);

    return CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Lifecycle: Activate
// -----------------------------------------------------------------------------

KinovaLifecycleController::CallbackReturn
KinovaLifecycleController::on_activate(const rclcpp_lifecycle::State& state)
{
    (void)state;

    // Establish connection to hardware
    auto connection = interface_->connect(robot_ip_, robot_port_);

    if (!connection) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to robot");
        return CallbackReturn::FAILURE;
    }

    // Activate publisher
    joint_state_pub_->on_activate();

    // Start periodic joint state publishing (10 Hz)
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { publishJointStates(); });

    RCLCPP_INFO(get_logger(), "Activation successful");
    return CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Lifecycle: Deactivate
// -----------------------------------------------------------------------------

KinovaLifecycleController::CallbackReturn
KinovaLifecycleController::on_deactivate(const rclcpp_lifecycle::State& state)
{
    (void)state;

    joint_state_pub_->on_deactivate();

    if (timer_) {
        timer_->cancel();
    }

    RCLCPP_INFO(get_logger(), "Deactivation successful");
    return CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Lifecycle: Cleanup
// -----------------------------------------------------------------------------

KinovaLifecycleController::CallbackReturn
KinovaLifecycleController::on_cleanup(const rclcpp_lifecycle::State& state)
{
    (void)state;

    // Disconnect and release interface
    if (interface_) {
        interface_->disconnect();
        interface_.reset();
    }

    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }

    joint_state_pub_.reset();
    action_server_.reset();

    RCLCPP_INFO(get_logger(), "Cleanup successful");
    return CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Lifecycle: Shutdown (Emergency)
// -----------------------------------------------------------------------------

KinovaLifecycleController::CallbackReturn
KinovaLifecycleController::on_shutdown(const rclcpp_lifecycle::State& state)
{
    (void)state;

    if (interface_) {
        interface_->emergencyStop();
        interface_->disconnect();
        interface_.reset();
    }

    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }

    if (joint_state_pub_) {
        joint_state_pub_.reset();
    }

    action_server_.reset();

    RCLCPP_INFO(get_logger(), "Shutdown successful");
    return CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Joint State Publisher
// -----------------------------------------------------------------------------

/**
 * @brief Periodically publish robot joint states.
 *
 * Reads joint angles from hardware and publishes them to /joint_states.
 * Runs only when node is ACTIVE.
 */
void KinovaLifecycleController::publishJointStates()
{
    try {
        auto joint_angles = interface_->getJointAngles();

        if (joint_angles.empty()) {
            RCLCPP_WARN(get_logger(), "Failed to read joint angles");
            return;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6", "joint_7"
        };
        msg.position = joint_angles;

        joint_state_pub_->publish(msg);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
            "publishJointStates exception: %s", e.what());
    }
}

// -----------------------------------------------------------------------------
// Action Server Callbacks
// -----------------------------------------------------------------------------

rclcpp_action::GoalResponse
KinovaLifecycleController::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const MoveToPose::Goal> goal)
{
    (void)uuid;
    (void)goal;

    // Reject if not connected or unsafe
    if (!interface_->isConnected()) {
        RCLCPP_WARN(get_logger(), "Goal rejected: not connected");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (interface_->isEStopActive()) {
        RCLCPP_WARN(get_logger(), "Goal rejected: e-stop active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
KinovaLifecycleController::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// -----------------------------------------------------------------------------
// Action Execution
// -----------------------------------------------------------------------------

/**
 * @brief Execute MoveToPose goal.
 *
 * Handles motion execution, feedback publishing, timeout, and cancellation.
 * Runs in a loop until goal completes, fails, or is canceled.
 */
void KinovaLifecycleController::execute_goal(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
    auto start_time = this->now();
    const double timeout_sec{10.0};

    geometry_msgs::msg::Pose current_pose, target_pose;

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    // Extract target
    target_pose = goal_handle->get_goal()->target_pose;

    // Initial state
    current_pose = toROSPose(interface_->getCurrentPose());
    double init_dist = compute_distance(target_pose, current_pose);

    // Send command
    auto target_wp = toWrapperPose(target_pose);
    auto future = interface_->moveToCartesianPoseAsync(target_wp);

    while (rclcpp::ok()) {

        // Timeout check
        if ((this->now() - start_time).seconds() > timeout_sec) {
            RCLCPP_WARN(get_logger(), "Motion timed out");

            result->success = false;
            result->message = "Timed out";
            result->final_pose = current_pose;
            result->execution_time =
                (this->now() - start_time).seconds();

            goal_handle->abort(result);
            return;
        }

        // Cancel handling
        if (goal_handle->is_canceling()) {
            interface_->emergencyStop();

            result->success = false;
            result->message = "Goal canceled";
            result->final_pose =
                toROSPose(interface_->getCurrentPose());
            result->execution_time = (this->now() - start_time).seconds();

            goal_handle->canceled(result);
            return;
        }

        current_pose = toROSPose(interface_->getCurrentPose());

        double distance = compute_distance(target_pose, current_pose);
        double percent = (init_dist > 0.001)
            ? (1.0 - distance / init_dist) * 100.0
            : 0.0;

        // Publish feedback
        feedback->current_pose = current_pose;
        feedback->distance_remaining = distance;
        feedback->percent_complete = percent;
        feedback->estimated_time_remaining = distance / 0.1;

        goal_handle->publish_feedback(feedback);

        // Goal reached
        if (distance < 0.005) {
            RCLCPP_INFO(get_logger(), "Goal reached");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Success result
    result->success = true;
    result->final_pose = current_pose;
    result->execution_time =
        (this->now() - start_time).seconds();
    result->message = "Goal reached";

    goal_handle->succeed(result);
}

} // namespace kinova_ros2

// -----------------------------------------------------------------------------
// Register as ROS 2 Component
// -----------------------------------------------------------------------------

RCLCPP_COMPONENTS_REGISTER_NODE(kinova_ros2::KinovaLifecycleController)