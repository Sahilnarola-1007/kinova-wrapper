#pragma once

// =============================================================================
// KinovaInterface.hpp — Production-grade wrapper around Kortex API
// =============================================================================
//
// Purpose:  Reduce 50+ lines of Kortex boilerplate to clean, safe, single-line calls.
// Pattern:  RAII — constructor initializes, destructor cleans up automatically.
// Thread:   All Kortex calls protected by a single mutex.
// Units:    Public API uses radians. Internal conversion to degrees (Kortex convention).
//
// Usage:
//   KinovaInterface kinova;
//   kinova.connect("192.168.1.10");
//   kinova.moveToJointAngles({0, 0.26, 0, -1.05, 0, -0.78, 0});
//   // destructor auto-cleans everything
// =============================================================================

#include <string>
#include <vector>
#include <memory>       // unique_ptr
#include <mutex>        // mutex, lock_guard
#include <atomic>       // atomic<bool>
#include <future>       // future, async
#include <cstdint>      // uint32_t

// Kortex SDK — use mock for local development, real SDK for hardware
#ifdef USE_KORTEX_MOCK
    #include "kinova_wrapper/kortex_mock.hpp"
#else
    #include <BaseClientRpc.h>
    #include <RouterClient.h>
    #include <TransportClientTcp.h>
    #include <SessionManager.h>
    namespace k_api = Kinova::Api;
#endif
#include "kinova_wrapper/Pose.hpp"

namespace kinova_wrapper {

    struct TrajectoryPoint {
    std::vector<double> joint_angles;  // radians
    double time_from_start;            // seconds from trajectory start
    };

    using MotionCallback=std::function<void(
        const std::vector<double>& current_joints,
        double progress)>;

class KinovaInterface {
public:
    // =========================================================================
    // Construction / Destruction
    // =========================================================================

    // Default constructor — does NOT connect. Connection is explicit via connect().
    KinovaInterface();

    // Destructor — calls disconnect(), which resets smart pointers in reverse order.
    // Must be noexcept-safe (destructors should never throw).
    ~KinovaInterface();

    // Non-copyable: this object owns a hardware connection.
    // Copying would mean two objects controlling one robot — dangerous.
    KinovaInterface(const KinovaInterface&) = delete;
    KinovaInterface& operator=(const KinovaInterface&) = delete;

    // Non-movable: mutex and atomics can't be moved.
    KinovaInterface(KinovaInterface&&) = delete;
    KinovaInterface& operator=(KinovaInterface&&) = delete;

    // =========================================================================
    // Connection
    // =========================================================================

    // Establish connection to Kinova Gen3.
    // Creates: TransportClientTcp → RouterClient → SessionManager → BaseClient
    // Also queries joint limits from robot firmware.
    // If already connected, disconnects first, then reconnects.
    // Returns: true if fully connected, false on any failure.
    bool connect(const std::string& ip_address, uint32_t port = 10000,
                 const std::string& username = "admin",
                 const std::string& password = "admin");

    // Cleanly close connection and release all resources.
    // Resets smart pointers in reverse creation order.
    // Safe to call multiple times. Safe to call if not connected.
    // Never throws — destructor calls this.
    void disconnect();

    // Check if robot connection is active.
    // Lock-free: reads atomic bool.
    bool isConnected() const;

    // =========================================================================
    // Motion
    // =========================================================================

    // Move robot to specified joint angles. Blocks until motion completes.
    // angles: exactly 7 values, in RADIANS.
    // Returns false if: not connected, e-stop active, wrong size, out of limits, motion error.
    bool moveToJointAngles(const std::vector<double>& angles);

    // Same as moveToJointAngles, but non-blocking.
    // Returns immediately with a future. Caller can poll or block on result.
    //   auto f = kinova.moveToJointAnglesAsync({...});
    //   // ... do other work ...
    //   bool ok = f.get();  // blocks until done
    std::future<bool> moveToJointAnglesAsync(const std::vector<double>& angles);

    // Move end-effector to a Cartesian position + orientation.
    // Kortex handles inverse kinematics internally.
    // Returns false on failure (unreachable pose, not connected, e-stop active).
    bool moveToCartesianPose(const Pose&pose);

    // Async version of moveToCartesianPose.
    std::future<bool> moveToCartesianPoseAsync(const Pose& pose);

    // =========================================================================
    // State Reading
    // =========================================================================

    // Read current joint positions from robot.
    // Returns: vector of 7 doubles in RADIANS. Empty vector on failure.
    // Returned by value — compiler applies RVO, no copy overhead.
    std::vector<double> getJointAngles();

    // Read current end-effector position and orientation.
    // Returns: Pose struct. Zeroed Pose on failure.
    Pose getCurrentPose();

    // Read forces (Fx, Fy, Fz) and torques (Tx, Ty, Tz) from built-in F/T sensor.
    // Returns: vector of 6 doubles. Empty vector on failure.
    std::vector<double> getWrench();

    // =========================================================================
    // Safety
    // =========================================================================

    // Immediately stop all robot motion.
    // Sets e_stop_active_ FIRST (atomic, instant), then calls Kortex e-stop.
    // Never throws — safety function must always attempt to stop.
    // All subsequent motion commands rejected until clearEmergencyStop().
    void emergencyStop();

    // To check if E stop is active or not
    bool isEStopActive() const;

    // Reset e-stop flag and allow commands again.
    // Returns false if ClearFaults fails (e-stop remains active).
    bool clearEmergencyStop();

    // Limit robot speed as a fraction of maximum. Range: [0.0, 1.0].
    // Returns false if out of range or not connected.
    bool setSpeedLimit(double fraction);

    // Gripper related functions
    bool openGripper(double speed=0.1);
    bool closeGripper(double force=40.0, double speed=0.1);
    bool setGripperPosition(double position, double speed=0.1);
    double getGripperPosition();
    bool isObjectDetected();

    #ifdef USE_KORTEX_MOCK
        k_api::Base::BaseClient* getBaseClientForTesting() { 
            return base_client_.get(); //Returns raw pointer- For testing mock 
        }
    #endif

    //Trajectory related methods
    bool executeTrajectory(const std::vector<TrajectoryPoint>& waypoints,
                            MotionCallback feedback_callback=nullptr);
       
    std::future<bool> executeTrajectoryAsync(const std::vector<TrajectoryPoint>& waypoints,
                            MotionCallback feedback_callback=nullptr);
private:
    // =========================================================================
    // Kortex API objects — owned via unique_ptr (RAII)
    // Creation order:  transport → router → session_manager → base_client
    // Destruction order: reverse (base_client → session_manager → router → transport)
    // =========================================================================
    std::unique_ptr<k_api::TransportClientTcp> transport_;
    std::unique_ptr<k_api::RouterClient>       router_;
    std::unique_ptr<k_api::SessionManager>     session_manager_;
    std::unique_ptr<k_api::Base::BaseClient>   base_client_;

    // =========================================================================
    // Thread safety
    // =========================================================================
    mutable std::mutex mutex_;              // protects all Kortex calls
    std::atomic<bool>  connected_{false};   // lock-free connection status
    std::atomic<bool>  e_stop_active_{false}; // lock-free e-stop flag

    // =========================================================================
    // Joint limits — queried from robot during connect()
    // Stored in DEGREES (Kortex convention). Conversion happens in public methods.
    // Joints 1,3,5,7: No limts
    // Joints 2,4,6: actual firmware limits
    // =========================================================================
    std::vector<double> joint_max_limits_{ 1e9, 128.9, 1e9, 1e9, 1e9, 120.3, 1e9};
    std::vector<double> joint_min_limits_{-1e9,-128.9,-1e9,-1e9,-1e9,-120.3,-1e9};
    // =========================================================================
    // Speed limit
    // =========================================================================
    double current_speed_fraction_ = 1.0;

    // =========================================================================
    // Private helpers
    // =========================================================================

    // Validate joint angles before sending to robot.
    // Checks: size == 7, each angle within [min, max] after radians→degrees conversion.
    // Returns true if valid.
    bool validateJointAngles(const std::vector<double>& angles) const;

    // Number of joints on Kinova Gen3
    static constexpr int kNumJoints = 7;

    // Conversion factor
    static constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    // assumes mutex_ already held
    void disconnectLocked();

    // Last commanded gripper position (for object detection comparison)
    double last_commanded_grip_pos_{0.0};

    // Gripper constants
    static constexpr double kGripperTimeoutSec = 5.0;
    static constexpr double kGripperPositionTolerance = 0.01;
    static constexpr double kMaxGripperForceN = 235.0;

    //Trajectory helper
    bool validateTrajectory(const std::vector<TrajectoryPoint> & waypoints) const;
};

}  // namespace kinova_wrapper
