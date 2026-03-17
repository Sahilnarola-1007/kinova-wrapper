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

    private:
        RouterClient* router_;  // non-owning
        bool e_stop_ = false;
        JointAngles current_joint_angles_;
        CartesianPose current_pose_;
        Wrench current_wrench_;
    };

}  // namespace Base
}  // namespace k_api
