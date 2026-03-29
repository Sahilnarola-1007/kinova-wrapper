#pragma once

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp> 
#include <sensor_msgs/msg/joint_state.hpp>
#include "kinova_interfaces/action/move_to_pose.hpp"
#include "kinova_interface/KinovaInterface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace kinova_ros2 {

/**
 * @class KinovaLifecycleController
 * @brief Lifecycle-managed ROS 2 node for controlling the Kinova Gen3 arm.
 *
 * This class implements a ROS 2 Lifecycle Node that manages the connection,
 * control, and state publishing of a Kinova robotic arm using explicit lifecycle
 * transitions. It wraps the low-level Kinova API through KinovaInterface and
 * exposes higher-level ROS 2 interfaces such as:
 *
 *   - Joint state publishing
 *   - Action-based motion control (MoveToPose)
 *
 * The lifecycle design ensures deterministic startup/shutdown behavior,
 * improved fault handling, and safer interaction with hardware.
 *
 * Lifecycle Overview:
 *   - on_configure(): Initialize resources and parameters
 *   - on_activate(): Connect to robot and start operation
 *   - on_deactivate(): Pause operation (keep connection alive)
 *   - on_cleanup(): Release resources and disconnect
 *   - on_shutdown(): Emergency shutdown and cleanup
 */
class KinovaLifecycleController : public rclcpp_lifecycle::LifecycleNode {
public:

    /// Alias for lifecycle callback return type
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /// Alias for MoveToPose action definition
    using MoveToPose = kinova_interfaces::action::MoveToPose;

    /// Alias for action server goal handle
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    /**
     * @brief Constructor for the lifecycle controller node.
     *
     * Initializes the node with given ROS 2 node options.
     * Resource allocation is deferred to on_configure().
     *
     * @param options ROS 2 node options (e.g., parameters, remapping).
     */
    explicit KinovaLifecycleController(const rclcpp::NodeOptions& options);

    // -------------------------------------------------------------------------
    // Lifecycle Transition Callbacks
    // -------------------------------------------------------------------------

    /**
     * @brief Configure the node.
     *
     * This transition is responsible for:
     *   - Declaring and retrieving parameters (e.g., robot IP, port)
     *   - Allocating core resources (interface, publishers, action server)
     *
     * NOTE: No hardware connection is established at this stage.
     *
     * @param state Current lifecycle state
     * @return SUCCESS if configuration succeeds, FAILURE otherwise
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

    /**
     * @brief Activate the node.
     *
     * This transition:
     *   - Establishes connection to the robot hardware
     *   - Activates lifecycle publishers
     *   - Starts periodic timers (e.g., joint state publishing)
     *
     * @param state Current lifecycle state
     * @return SUCCESS if activation succeeds, FAILURE otherwise
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

    /**
     * @brief Deactivate the node.
     *
     * This transition:
     *   - Stops timers and periodic execution
     *   - Deactivates publishers
     *
     * NOTE: Hardware connection is preserved for fast reactivation.
     *
     * @param state Current lifecycle state
     * @return SUCCESS if deactivation succeeds, FAILURE otherwise
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

    /**
     * @brief Cleanup the node.
     *
     * This transition:
     *   - Disconnects from robot hardware
     *   - Destroys allocated resources (publishers, interfaces)
     *
     * After this, the node can safely transition back to "unconfigured".
     *
     * @param state Current lifecycle state
     * @return SUCCESS if cleanup succeeds, FAILURE otherwise
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

    /**
     * @brief Shutdown the node.
     *
     * This transition performs an emergency shutdown:
     *   - Triggers robot safety mechanisms (e.g., e-stop if applicable)
     *   - Releases all resources
     *
     * This is typically called when the system is terminating.
     *
     * @param state Current lifecycle state
     * @return SUCCESS if shutdown succeeds, FAILURE otherwise
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:

    // -------------------------------------------------------------------------
    // Core Robot Interface
    // -------------------------------------------------------------------------

    /**
     * @brief Wrapper around Kinova Kortex API.
     *
     * Lifecycle:
     *   - Created in on_configure()
     *   - Connected in on_activate()
     *   - Destroyed in on_cleanup()
     */
    std::unique_ptr<kinova_wrapper::KinovaInterface> interface_;

    // -------------------------------------------------------------------------
    // Publishers and Timers
    // -------------------------------------------------------------------------

    /**
     * @brief Lifecycle-aware publisher for joint states.
     *
     * Publishes sensor_msgs/JointState messages when the node is ACTIVE.
     */
    rclcpp_lifecycle::LifecyclePublisher
        <sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    /**
     * @brief Timer for periodic execution.
     *
     * Triggers joint state publishing at a fixed rate (e.g., 10 Hz).
     */
    rclcpp::TimerBase::SharedPtr timer_;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------

    /// Robot IP address (loaded from ROS 2 parameters)
    std::string robot_ip_{"0.0.0.0"};

    /// Robot port number (default: 10000)
    uint32_t robot_port_{10000};

    // -------------------------------------------------------------------------
    // Action Server (MoveToPose)
    // -------------------------------------------------------------------------

    /**
     * @brief Action server for handling MoveToPose goals.
     *
     * Provides asynchronous motion execution interface to clients.
     */
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    /**
     * @brief Callback for incoming goal requests.
     *
     * Validates and decides whether to accept or reject a goal.
     * This does NOT execute the goal.
     *
     * @param uuid Unique identifier for the goal
     * @param goal Requested goal
     * @return GoalResponse (ACCEPT_AND_EXECUTE / REJECT)
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MoveToPose::Goal> goal);

    /**
     * @brief Callback for cancel requests.
     *
     * Determines whether an active goal should be canceled.
     *
     * @param goal_handle Handle to the active goal
     * @return CancelResponse (ACCEPT / REJECT)
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

    /**
     * @brief Executes an accepted goal.
     *
     * Contains the main motion execution logic:
     *   - Sends commands to the robot
     *   - Publishes feedback periodically
     *   - Sets final result (success, aborted, canceled)
     *
     * Typically executed in a separate thread to avoid blocking the executor.
     *
     * @param goal_handle Handle used for feedback and result reporting
     */
    void execute_goal(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle
    );

    // -------------------------------------------------------------------------
    // Internal Utility Functions
    // -------------------------------------------------------------------------

    /**
     * @brief Publish current joint states.
     *
     * Reads joint angles from the robot via KinovaInterface and publishes
     * them to the /joint_states topic.
     *
     * Called periodically by the timer when the node is ACTIVE.
     */
    void publishJointStates();
};

}  // namespace kinova_ros2