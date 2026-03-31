#pragma once

// =============================================================================
// Mock Kortex API Types
// =============================================================================
// These stubs mirror the real Kortex SDK object hierarchy:
//   TransportClientTcp → RouterClient → SessionManager → BaseClient
//
// Purpose: Let us compile and test wrapper logic without the real SDK.
// On your lab machine, replace this with:
//   #include <KortexApiClient.h>
//   #include <BaseClientRpc.h>
//   etc.
// =============================================================================

#include <string>
#include <vector>
#include <stdexcept>

namespace k_api {

// --- Transport Layer ---
class TransportClientTcp {
public:
    void connect(const std::string& ip, uint32_t port) {
        // Real SDK: opens TCP socket to robot
        if (ip.empty()) {
            throw std::runtime_error("Invalid IP address");
        }
        connected_ = true;
    }

    void disconnect() {
        connected_ = false;
    }

    bool isConnected() const { return connected_; }

private:
    bool connected_ = false;
};

// --- Router Layer ---
class RouterClient {
public:
    // Real SDK: takes TransportClient* (non-owning pointer)
    explicit RouterClient(TransportClientTcp* transport)
        : transport_(transport) {}

private:
    TransportClientTcp* transport_;  // non-owning
};

// --- Session Layer ---
struct CreateSessionInfo {
    std::string username;
    std::string password;
    uint32_t session_timeout_ms = 60000;
};

class SessionManager {
public:
    // Real SDK: takes RouterClient* (non-owning pointer)
    explicit SessionManager(RouterClient* router)
        : router_(router) {}

    void CreateSession(const CreateSessionInfo& info) {
        if (info.username.empty() || info.password.empty()) {
            throw std::runtime_error("Authentication failed");
        }
        session_active_ = true;
    }

    void CloseSession() {
        session_active_ = false;
    }

    bool isSessionActive() const { return session_active_; }

private:
    RouterClient* router_;  // non-owning
    bool session_active_ = false;
};

// --- Base Client (main control interface) ---
namespace Base {

    // Gripper control modes
    constexpr uint32_t GRIPPER_POSITION = 0;
    constexpr uint32_t GRIPPER_SPEED = 1;
    constexpr uint32_t GRIPPER_FORCE = 2;

    // Simulated joint angle measurement from robot
    struct JointAngle {
        uint32_t joint_identifier = 0;
        double value = 0.0;  // degrees (Kortex convention)
    };

    struct JointAngles {
        std::vector<JointAngle> joint_angles;
    };

    // Simulated Cartesian pose from robot
    struct CartesianPose {
        double x = 0.0, y = 0.0, z = 0.0;           // meters
        double theta_x = 0.0, theta_y = 0.0, theta_z = 0.0;  // degrees
    };

    // Simulated wrench (force/torque) from F/T sensor
    struct Wrench {
        double force_x = 0.0, force_y = 0.0, force_z = 0.0;
        double torque_x = 0.0, torque_y = 0.0, torque_z = 0.0;
    };

    // Simulated joint limit info
    struct JointLimitInfo {
        double min_value = 0.0;  // degrees
        double max_value = 0.0;  // degrees
    };

    // Simulated action for motion commands
    struct Action {
        JointAngles target_joint_angles;
        CartesianPose target_pose;
        bool is_joint_action = false;
        bool is_cartesian_action = false;
    };

    // Gripper finger — single actuator data
    struct Finger {
        uint32_t finger_identifier = 0;
        double value = 0.0;
    };

    // Gripper — contains list of fingers
    struct Gripper {
        std::vector<Finger> finger;
    };

    // Gripper command — what you send
    struct GripperCommand {
        uint32_t mode = 0;
        Gripper gripper;
    };

    // Gripper request — what you ask for when reading
    struct GripperRequest {
        uint32_t mode = 0;
    };

    class BaseClient {
    public:
        explicit BaseClient(RouterClient* router)
            : router_(router) {
            // Real robot always has joint positions — initialize with 7 zeros
            for (uint32_t i = 0; i < 7; ++i) {
                JointAngle ja;
                ja.joint_identifier = i;
                ja.value = 0.0;
                current_joint_angles_.joint_angles.push_back(ja);
            }
        }

        // --- Motion ---
        void ExecuteAction(const Action& action) {
            // Real SDK: sends protobuf action to robot, blocks until done
            if (e_stop_) {
                throw std::runtime_error("Emergency stop is active");
            }
            // Simulate: store last commanded values
            if (action.is_joint_action) {
                current_joint_angles_ = action.target_joint_angles;
            }
        }

        // --- State Reading ---
        JointAngles GetMeasuredJointAngles() {
            return current_joint_angles_;
        }

        CartesianPose GetMeasuredCartesianPose() {
            return current_pose_;
        }

        Wrench GetMeasuredWrench() {
            return current_wrench_;
        }

        // --- Joint Limits ---
        std::vector<JointLimitInfo> GetJointLimits() {
            // Real SDK: queries robot firmware for actual limits
            // Gen3 7DOF typical limits (degrees):
            //   Joints 1,3,5,7: continuous (-inf to +inf) → use ±360
            //   Joints 2,4,6: limited to prevent self-collision
            return {
                {-360.0, 360.0},   // Joint 1: continuous
                {-128.9, 128.9},   // Joint 2: limited
                {-360.0, 360.0},   // Joint 3: continuous
                {-147.8, 147.8},   // Joint 4: limited
                {-360.0, 360.0},   // Joint 5: continuous
                {-120.3, 120.3},   // Joint 6: limited
                {-360.0, 360.0}    // Joint 7: continuous
            };
        }

        // --- Safety ---
        void ApplyEmergencyStop() {
            e_stop_ = true;
        }

        void ClearFaults() {
            e_stop_ = false;
        }

        // Gripper methods
        void setSimulateObject(bool val) { simulate_object_ = val; }

        void SendGripperCommand(const GripperCommand& cmd) {
            if (!cmd.gripper.finger.empty()) {
                stored_gripper_position_ = cmd.gripper.finger[0].value;
            }
        }

        Gripper GetMeasuredGripperMovement(const GripperRequest& req) {
            (void)req;
            Gripper g;
            Finger f;
            f.finger_identifier = 1;

            // Simulate object stall: gripper can't fully close
            if (simulate_object_ && stored_gripper_position_ > 0.5) {
                f.value = 0.4;  // stalled at 0.4 instead of reaching 1.0
            } else {
                f.value = stored_gripper_position_;  // instant arrival
            }

            g.finger.push_back(f);
            return g;
        }

    private:
        RouterClient* router_;  // non-owning
        bool e_stop_ = false;
        JointAngles current_joint_angles_;
        CartesianPose current_pose_;
        Wrench current_wrench_;
        double stored_gripper_position_ = 0.0; // tracks position 
        bool simulate_object_ = false;

    };

}  // namespace Base
}  // namespace k_api
