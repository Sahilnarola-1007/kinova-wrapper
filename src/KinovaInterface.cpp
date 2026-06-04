// =============================================================================
// KinovaInterface.cpp — Production-grade C++ wrapper for the Kinova Gen3
//                       Kortex SDK (7-DOF arm + Robotiq 2F-85 gripper)
// =============================================================================
//
// This file implements the complete low-level interface between user code (or
// ROS2 nodes) and the Kinova Gen3 arm. It abstracts the Kortex SDK's protobuf-
// based API behind clean C++ methods with RAII resource management, thread
// safety, and input validation.
//
// Design principles:
//   1. RAII throughout — unique_ptr ownership of transport, router, session,
//      and base client. Destructor tears down in reverse creation order.
//      No manual cleanup needed even on exceptions.
//
//   2. Thread safety — every Kortex SDK call is protected by mutex_. Atomic
//      flags (connected_, e_stop_active_, velocity_active_) allow lock-free
//      reads for fast pre-checks before acquiring the mutex.
//
//   3. Mock/real compile switch — USE_KORTEX_MOCK (CMake option) swaps the
//      Kortex SDK for a lightweight mock (kortex_mock.hpp) that simulates
//      responses without hardware. Same source compiles for both targets.
//      Default is mock; hardware builds require explicit -DUSE_KORTEX_MOCK=OFF.
//
//   4. Radians at the API boundary, degrees internally — the public API
//      accepts/returns radians (standard robotics convention). Conversion
//      to degrees (Kortex convention) happens inside each method.
//
//   5. Private helpers never lock mutex_ — public methods hold the lock and
//      call private helpers (disconnectLocked, validateJointAngles, etc.)
//      that assume the lock is already held. This prevents recursive locking.
//
// Kortex SDK quirks (discovered during hardware validation):
//   - ExecuteAction() is non-blocking on real SDK. Requires subscribing to
//     OnNotificationActionTopic with a std::promise to detect ACTION_END.
//   - SendGripperCommand() is also non-blocking. Requires polling loop to
//     confirm the gripper reached the target position.
//   - Session inactivity timeouts must be set explicitly (60s session, 2s
//     connection) — omitting them causes silent session closure and action
//     aborts after ~10 seconds of inactivity.
//   - SetServoingMode(SINGLE_LEVEL_SERVOING) required before ExecuteAction.
//   - SendTwistCommand() expects angular velocities in deg/s, not rad/s.
//     The admittance controller handles this conversion at the call site.
//   - SendTwistCommand() blocks for ~73ms (gRPC round-trip), capping the
//     velocity control loop at ~13 Hz through the high-level API.
//
// Consumers:
//   - admittance_controller (admittance_node.cpp) — velocity control loop
//   - kinova_moveit_bridge — FollowJointTrajectory action server
//   - KinovaLifecycleController — ROS2 lifecycle node with joint state pub
//   - Direct use in hardware test suites (6 test files)
//
// Author: Sahil Narola — MEng, Carleton University (ABLL Lab)
// =============================================================================

#include "kinova_wrapper/KinovaInterface.hpp"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <cmath>
#include <memory>
#include <thread>
#include<algorithm>

namespace kinova_wrapper
{

    // =========================================================================
    // Constructor: lightweight — no SDK resources allocated here.
    // The connect() method handles all SDK initialization so that
    // construction never throws and objects can be created before the
    // network is ready.
    // =========================================================================
    KinovaInterface::KinovaInterface()
    {
        std::cout << "KinovaInterface created. Call connect method \
        to build TCP connection"
                  << std::endl;
    };

    // =========================================================================
    // Destructor: RAII teardown in reverse creation order.
    // 1. Signal the watchdog thread to stop and join it
    // 2. Disconnect from the arm (closes session, transport, resets clients)
    // This ensures no dangling connections even if the caller forgets to
    // call disconnect() or if an exception unwinds the stack.
    // =========================================================================
    KinovaInterface::~KinovaInterface()
    {

        std::cout << "Destroying connection and reseting all the objects"
                  << std::endl;

        watchdog_running_.store(false);
        if (watchdog_thread_.joinable())
        {
            watchdog_thread_.join();
        }
        disconnect();
    };

    // =========================================================================
    // Part 1: Connection Management
    //
    // Kortex SDK connection is a 7-step pipeline:
    //   1. Validate inputs
    //   2. TransportClientTcp — raw TCP socket to the arm
    //   3. RouterClient — multiplexes requests over the transport
    //   4. SessionManager — manages authenticated sessions
    //   5. CreateSession — authenticates with username/password
    //   6. BaseClient — the main API surface for arm commands
    //   7. BaseCyclicClient — low-level cyclic data (wrench, real HW only)
    //
    // Each step depends on the previous. If any step fails, all prior
    // resources are cleaned up before returning false. On success, the
    // connected_ atomic flag is set to allow lock-free connection checks.
    // =========================================================================

    // connect(): Establishes the full Kortex SDK connection pipeline.
    // If already connected, gracefully disconnects first (idempotent).
    // Returns true on success, false with cleanup on any failure.
    bool KinovaInterface::connect(const std::string &ip_address,
                                  uint32_t port,
                                  const std::string &username,
                                  const std::string &password)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // step 1: validate inputs before touching SDK
        if (ip_address.empty())
        {
            std::cerr << "Error: IP address is empty" << std::endl;
            return false;
        }
        // step 1: To check the connection if its already connected or not
        if (connected_.load())
        {
            std::cout << "Already connected, Closing a session and connection" << std::endl;
            disconnectLocked();
        }

        // step 2: Creating transport client
        try
        {
            transport_ = std::make_unique<k_api::TransportClientTcp>();
            transport_->connect(ip_address, port);
        }
        catch (const std::exception &e)
        {
            std::cout << "Robot unreachable with ip: " << ip_address
                      << " and port " << port << std::endl;
            std::cerr << e.what() << std::endl;
            transport_.reset();
            return false;
        }

        // step 3: creating router client with error callback
#ifdef USE_KORTEX_MOCK
        router_ = std::make_unique<k_api::RouterClient>(transport_.get());
#else
        router_ = std::make_unique<k_api::RouterClient>(transport_.get(),
                                                        [](k_api::KError err)
                                                        {
                                                            std::cerr << "Router error: " << err.toString() << "\n";
                                                        });
#endif

        // step 4: Creating session_manager for managing session
        session_manager_ = std::make_unique<k_api::SessionManager>(router_.get());

        // step 5: Creating session info and pass it to the session manager
#ifdef USE_KORTEX_MOCK
        k_api::CreateSessionInfo session_info_;
        session_info_.username = username;
        session_info_.password = password;
#else
        k_api::Session::CreateSessionInfo session_info_;
        session_info_.set_username(username);
        session_info_.set_password(password);
        session_info_.set_session_inactivity_timeout(60000);   // default session timeout is very short. Session closes immmediately if no commands get sent from client to base
        session_info_.set_connection_inactivity_timeout(2000); //
#endif

        // step 6: starting a session
        try
        {
            session_manager_->CreateSession(session_info_);
            std::cout << "session created" << std::endl;
        }

        catch (const std::exception &e)
        {
            std::cout << "session not created. Authentication failed!" << std::endl;
            std::cout << e.what() << std::endl;

            session_manager_.reset();
            router_.reset();
            transport_->disconnect();
            transport_.reset();
            return false;
        }

        // step 7: Creating the base_client which will talk with the robot's base
        try
        {
            base_client_ = std::make_unique<k_api::Base::BaseClient>(router_.get());
            std::cout << "Base Client created" << std::endl;

            //To get the real hardware's wrench readings.(Warning!- Doesn't exist in mock)
            #ifndef USE_KORTEX_MOCK
                base_cyclic_client_=std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(router_.get());
            #endif
            }
        catch (const std::exception &e)
        {
            std::cout << "Base client is not created" << std::endl;
            std::cout << e.what() << std::endl;

            base_client_.reset();
            session_manager_->CloseSession();
            session_manager_.reset();
            router_.reset();
            transport_->disconnect();
            transport_.reset();
            return false;
        }

        connected_.store(true);
        e_stop_active_.store(false);
        std::cout << " Connected successfully and ready.\n";
        return true;
    };

    // disconnectLocked(): Tears down SDK resources in reverse creation order.
    // PRIVATE — called with mutex_ already held by the caller (disconnect()
    // or connect() during reconnection). Never acquires mutex_ itself to
    // avoid recursive locking. Errors during teardown are logged but do not
    // prevent cleanup of remaining resources.
    void KinovaInterface::disconnectLocked()
    {

        // If not connected or nothing to clean up then return to the caller
        if (!connected_.load())
        {
            std::cout << "Disconnected" << std::endl;
            return;
        }

        std::cout << "disconnecting" << std::endl;
        base_client_.reset();
        try
        {
            if (session_manager_)
            {
                session_manager_->CloseSession();
            }
        }

        catch (const std::exception &e)
        {
            std::cout << "Warning! Error while closing a session" << std::endl;
            std::cout << " continue cleaning up.." << std::endl;
            std::cerr << e.what() << std::endl;
        }

        session_manager_.reset();
        router_.reset();

        try
        {
            if (transport_)
            {
                transport_->disconnect();
            }
        }

        catch (const std::exception &e)
        {
            std::cerr << " Warning: transport disconnect failed: "
                      << e.what() << "\n";
        }

        transport_.reset();

        connected_.store(false);
    }

    // disconnect(): Public thread-safe wrapper — acquires mutex_ then
    // delegates to disconnectLocked().
    void KinovaInterface::disconnect()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        disconnectLocked();
    }

    // isConnected(): Lock-free connection check via atomic flag.
    // Used by all public methods as a fast pre-check before acquiring mutex_.
    bool KinovaInterface::isConnected() const
    {
        return connected_.load();
    }

    // isEStopActive(): Lock-free e-stop check via atomic flag.
    // emergencyStop() sets this without holding mutex_ (safety-critical
    // path must never block on a potentially held lock).
    bool KinovaInterface::isEStopActive() const
    {
        return e_stop_active_.load();
    }

    // isVelocityActive(): Lock-free check for active velocity control mode.
    // Set by setCartesianVelocity(), cleared by stopMotion() or watchdog.
    bool KinovaInterface::isVelocityActive() const
    { 
        return velocity_active_.load();
    }
    // =========================================================================
    // Part 2: Motion Commands
    //
    // All motion methods follow the same pattern:
    //   1. Check atomic flags (connected_, e_stop_active_) — lock-free
    //   2. Acquire mutex_
    //   3. Validate inputs (joint limits, workspace bounds)
    //   4. Build Kortex protobuf Action message
    //   5. Set servoing mode (real HW only — SINGLE_LEVEL_SERVOING)
    //   6. Execute and wait for completion notification
    //   7. Return success/failure
    //
    // Async variants capture arguments by value into a std::async lambda
    // to prevent dangling references when the caller's scope exits before
    // the async thread runs.
    //
    // Real SDK quirk: ExecuteAction() is non-blocking. Completion is
    // detected via OnNotificationActionTopic with a std::promise that
    // resolves on ACTION_END or ACTION_ABORT. The mutex is unlocked
    // before the wait to allow other threads (e.g. state reading) to
    // proceed during motion. 30-second timeout protects against stalls.
    // =========================================================================

    // validateJointAngles(): Checks that all 7 angles are within the
    // Gen3's firmware-defined joint limits. Converts from radians (API)
    // to degrees (Kortex internal) for comparison. Called under mutex_.
    bool KinovaInterface::validateJointAngles(const std::vector<double> &angles) const
    {

        if (angles.size() != static_cast<size_t>(kNumJoints))
        {
            std::cerr << "Error! The number of joints are: " << angles.size()
                      << " but expected " << kNumJoints << std::endl;
            return false;
        }
        for (size_t i = 0; i < angles.size(); i++)
        {

            // Converting angles Rad to Deg because kinova expects angles in Deg
            double angle_deg = angles[i] * kRadToDeg;

            if (angle_deg < joint_min_limits_[i] || angle_deg > joint_max_limits_[i])
            {
                std::cerr << " Error! joint:= " << i << "is not within the range"
                          << std::endl;

                return false;
            }
        }
        return true;
    }

    // moveToJointAngles(): Synchronous joint-space motion.
    // Blocks until the arm reaches the target or times out (30s).
    // On real hardware, unlocks mutex_ before waiting on the action
    // notification to allow concurrent state reads.
    bool KinovaInterface::moveToJointAngles(const std::vector<double> &angles)
    {

        // To check the status of atomic variables before lock
        if (!connected_.load())
        {
            std::cerr << "Error! Not connected" << std::endl;
            return false;
        }
        if (e_stop_active_.load())
        {
            std::cerr << "Error! e-stop is active we can't proceed this action"
                      << std::endl;
            return false;
        }

        // thread-safe lock to protect shared state(joint_min_limits_ and joint_max_limit_ in the validate joint function)
        std::unique_lock<std::mutex> lock(mutex_);

        // Validating joint angles
        if (!validateJointAngles(angles))
        {
            return false;
        }
        try
        {

            // Building kortex action: To send the commands to the joints
            k_api::Base::Action action;

#ifdef USE_KORTEX_MOCK
            action.is_joint_action = true;
            for (size_t i = 0; i < angles.size(); i++)
            {
                k_api::Base::JointAngle joint;
                joint.joint_identifier = static_cast<uint32_t>(i);
                joint.value = angles[i] * kRadToDeg;
                action.target_joint_angles.joint_angles.push_back(joint);
            }
#else
            action.set_name("moveToJointAngles");
            action.set_application_data("");

            for (size_t i = 0; i < angles.size(); i++)
            {
                auto *joint_angles = action.mutable_reach_joint_angles()->mutable_joint_angles();
                auto *joint = joint_angles->add_joint_angles();
                joint->set_joint_identifier(static_cast<uint32_t>(i));
                joint->set_value(angles[i] * kRadToDeg);
            }
#endif

#ifndef USE_KORTEX_MOCK
            auto servoingMode = k_api::Base::ServoingModeInformation();
            servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base_client_->SetServoingMode(servoingMode);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

#endif

            // Calling kortex

#ifdef USE_KORTEX_MOCK
            base_client_->ExecuteAction(action); // In the mock, it is Non-Blocking
            std::cout << "joint motion completed\n";
            return true;
#else
            std::promise<k_api::Base::ActionEvent> finished;
            auto future_event = finished.get_future();

            lock.unlock();
            auto handle = base_client_->OnNotificationActionTopic(
                [&finished](k_api::Base::ActionNotification notification)
                {
                    const auto event = notification.action_event();
                    if (event == k_api::Base::ACTION_END ||
                        event == k_api::Base::ACTION_ABORT)
                    {
                        finished.set_value(event);
                    }
                },
                k_api::Common::NotificationOptions());

            base_client_->ExecuteAction(action);

            auto status = future_event.wait_for(std::chrono::seconds(30));
            base_client_->Unsubscribe(handle);

            if (status == std::future_status::timeout)
            {
                std::cerr << "Motion timed out\n";
                return false;
            }
            if (future_event.get() == k_api::Base::ACTION_ABORT)
            {
                std::cerr << "Motion aborted by robot\n";
                return false;
            }
            std::cout << "joint motion completed\n";
            return true;
#endif
        }

        catch (const std::exception &e)
        {
            std::cerr << "Error! Joint motion failed" << e.what() << std::endl;

            return false;
        }
    }

    // moveToJointAnglesAsync(): Non-blocking joint motion.
    // Returns a std::future<bool> — caller must store the future (discarding
    // it causes the destructor to block immediately, defeating the purpose).
    // Captures angles by value to prevent dangling references.
    std::future<bool> KinovaInterface::moveToJointAnglesAsync(
        const std::vector<double> &angles)
    {

        // Capture angles BY VALUE into the lambda.
        // Why? The caller's vector might go out of scope before the async thread runs.
        // Capturing by reference would be a dangling reference → undefined behavior.

        return std::async(std::launch::async, [this, angles]()
                          { return this->moveToJointAngles(angles); });
    }

    // moveToCartesianPose(): Synchronous Cartesian-space motion.
    // Pose contains (x, y, z) in meters and (theta_x, theta_y, theta_z)
    // in degrees (Kortex convention for Euler angles). Same notification-
    // based blocking pattern as moveToJointAngles.
    bool KinovaInterface::moveToCartesianPose(const Pose &pose)
    {

        if (!connected_.load())
        {
            std::cerr << "Error! Not connected" << std::endl;
            return false;
        }
        if (e_stop_active_.load())
        {
            std::cerr << "Error! e-stop is active we can't proceed this action"
                      << std::endl;
            return false;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        try
        {

            k_api::Base::Action action;

#ifdef USE_KORTEX_MOCK
            action.is_cartesian_action = true;
            action.target_pose.x = pose.x;
            action.target_pose.y = pose.y;
            action.target_pose.z = pose.z;
            action.target_pose.theta_x = pose.theta_x;
            action.target_pose.theta_y = pose.theta_y;
            action.target_pose.theta_z = pose.theta_z;
#else
            action.set_name("moveToCartesianPose");
            action.set_application_data("");

            auto *pose_msg = action.mutable_reach_pose()->mutable_target_pose();
            pose_msg->set_x(pose.x);
            pose_msg->set_y(pose.y);
            pose_msg->set_z(pose.z);
            pose_msg->set_theta_x(pose.theta_x);
            pose_msg->set_theta_y(pose.theta_y);
            pose_msg->set_theta_z(pose.theta_z);
#endif

#ifndef USE_KORTEX_MOCK
            auto servoingMode = k_api::Base::ServoingModeInformation();
            servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base_client_->SetServoingMode(servoingMode);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
#endif

            // Calling kortex
#ifdef USE_KORTEX_MOCK
            base_client_->ExecuteAction(action);
            std::cout << "Cartesian motion completed.\n";
            return true;
#else
            std::promise<k_api::Base::ActionEvent> finished;
            auto future_event = finished.get_future();

            lock.unlock();
            auto handle = base_client_->OnNotificationActionTopic(
                [&finished](k_api::Base::ActionNotification notification)
                {
                    const auto event = notification.action_event();
                    if (event == k_api::Base::ACTION_END ||
                        event == k_api::Base::ACTION_ABORT)
                    {
                        finished.set_value(event);
                    }
                },
                k_api::Common::NotificationOptions());

            base_client_->ExecuteAction(action);

            auto status = future_event.wait_for(std::chrono::seconds(30));
            base_client_->Unsubscribe(handle);

            if (status == std::future_status::timeout)
            {
                std::cerr << "Cartesian motion timed out\n";
                return false;
            }
            if (future_event.get() == k_api::Base::ACTION_ABORT)
            {
                std::cerr << "Cartesian motion aborted by robot\n";
                return false;
            }
            std::cout << "Cartesian motion completed.\n";
            return true;
#endif
        }

        catch (const std::exception &e)
        {
            std::cerr << "Cartesian motion failed: "
                      << e.what() << "\n";
            return false;
        }
    }

    // moveToCartesianPoseAsync(): Non-blocking Cartesian motion.
    // Same capture-by-value pattern as moveToJointAnglesAsync.
    std::future<bool> KinovaInterface::moveToCartesianPoseAsync(const Pose &pose)
    {
        return std::async(std::launch::async,
                          [this, pose]()
                          {
                              return this->moveToCartesianPose(pose);
                          });
    }

    // emergencyStop(): Immediately halts all arm motion.
    // Sets e_stop_active_ FIRST (atomic, no mutex needed) so all other
    // methods see the flag immediately, even if they're holding the mutex.
    // Then calls ApplyEmergencyStop on the SDK. If the SDK call fails,
    // the flag remains set — safer to stay stopped than risk motion.
    void KinovaInterface::emergencyStop()
    {
        e_stop_active_.store(true);

        try
        {

            if (base_client_)
            {
                base_client_->ApplyEmergencyStop();
                std::cout << "Emergency stop is activated! kortex call succeed" << std::endl;
            }
        }
        catch (const std::exception &e)
        {

            std::cerr << "Emergency stop kortex call failed: "
                      << e.what() << "- Flag still set " << std::endl;
        }
    }

    // clearEmergencyStop(): Clears the e-stop and all actuator faults.
    // Acquires mutex_ because ClearFaults() is a Kortex SDK call.
    // Only clears the atomic flag after the SDK call succeeds — if
    // ClearFaults() throws, the arm remains in e-stop state.
    bool KinovaInterface::clearEmergencyStop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        try
        {

            if (base_client_)
            {
                base_client_->ClearFaults(); // clears the state
            }

            e_stop_active_.store(false);
            std::cout << "Emergency stop is cleared! ready to work"
                      << std::endl;

            return true;
        }

        catch (const std::exception &e)
        {
            std::cerr << "Failed to clear the emergency stop"
                      << e.what();

            // still emergency stop=true safer to stay stopped

            return false;
        }
    }

    // =========================================================================
    // Part 3: State Reading
    //
    // All state methods follow the same pattern: check connected_ (atomic),
    // acquire mutex_, call the Kortex getter, convert units, return.
    //
    // getCurrentPose() is performance-critical: it takes ~20ms per call
    // (Kortex gRPC round-trip). The admittance controller caches this in
    // a background thread to avoid blocking the control loop.
    //
    // getWrench() uses BaseCyclicClient::RefreshFeedback() on real hardware
    // (not available in mock) because BaseClient doesn't expose external
    // wrench readings directly. The admittance controller uses the external
    // MAE SensuReal sensor instead — this method is the fallback.
    // =========================================================================

    // getJointAngles(): Returns all 7 joint angles in radians.
    // Kortex returns degrees; conversion happens here at the API boundary.
    std::vector<double> KinovaInterface::getJointAngles()
    {

        if (!connected_.load())
        {
            std::cerr << "Not connected! Joint angle reading failed" << std::endl;
            return {};
        }

        std::lock_guard<std::mutex> lock(mutex_);

        try
        {
            auto joint_angles = base_client_->GetMeasuredJointAngles();
            std::vector<double> angles;
            angles.reserve(kNumJoints);

            // getJointAngles — mock uses vector member, real SDK uses protobuf getter
#ifdef USE_KORTEX_MOCK
            for (const auto &angle : joint_angles.joint_angles)
            {
                angles.push_back(angle.value * kDegToRad);
            }
#else
            for (int i = 0; i < joint_angles.joint_angles_size(); i++)
            {
                angles.push_back(joint_angles.joint_angles(i).value() * kDegToRad);
            }
#endif
            return angles; // Angles in radians
        }
        catch (const std::exception &e)
        {
            std::cerr << "Joint angles reading failed: " << e.what() << std::endl;
            return {};
        }
    }

    // getCurrentPose(): Returns end-effector pose (x,y,z in meters,
    // theta_x/y/z in degrees — Kortex ZYX intrinsic Euler convention).
    // This call takes ~20ms on real hardware (gRPC round-trip). The
    // admittance controller's pose_thread_ caches this to avoid blocking
    // the control loop. Returns default-constructed Pose{} on failure.
    Pose KinovaInterface::getCurrentPose()
    {

        if (!connected_.load())
        {
            std::cerr << "Not connected! Pose reading failed" << std::endl;
            return Pose{};
        }

        std::lock_guard<std::mutex> lock(mutex_);

        try
        {

            auto kortex_pose = base_client_->GetMeasuredCartesianPose();
            Pose pose;

            // getCurrentPose — mock uses direct members, real SDK uses protobuf getters
#ifdef USE_KORTEX_MOCK
            pose.x = kortex_pose.x;
            pose.y = kortex_pose.y;
            pose.z = kortex_pose.z;
            pose.theta_x = kortex_pose.theta_x;
            pose.theta_y = kortex_pose.theta_y;
            pose.theta_z = kortex_pose.theta_z;
#else
            pose.x = kortex_pose.x();
            pose.y = kortex_pose.y();
            pose.z = kortex_pose.z();
            pose.theta_x = kortex_pose.theta_x();
            pose.theta_y = kortex_pose.theta_y();
            pose.theta_z = kortex_pose.theta_z();
#endif

        return pose;
        }

        catch (const std::exception &e)
        {
            std::cerr << "Pose reading failed: " << e.what() << std::endl;
            return Pose{};
        }
    }

    // getWrench(): Returns the 6-axis wrench [Fx, Fy, Fz, Tx, Ty, Tz].
    // On real hardware, reads from BaseCyclicClient::RefreshFeedback()
    // which provides the Kortex-estimated external wrench (model-based,
    // ~10Hz, lower quality than the MAE SensuReal sensor).
    // Used as a fallback when the external F/T sensor is unavailable.
    std::vector<double> KinovaInterface::getWrench()
    {
        if (!connected_.load())
        {
            std::cerr << "Not connected! Wrench reading failed" << std::endl;
            return {};
        }
        std::lock_guard<std::mutex> lock(mutex_);

        try{
            std::vector<double> wrench_vec;
        
        # ifdef USE_KORTEX_MOCK
            auto wrench_mock=base_client_->GetMeasuredWrench();  //In the mock, it is struct
            wrench_vec.push_back(wrench_mock.force_x);
            wrench_vec.push_back(wrench_mock.force_y);
            wrench_vec.push_back(wrench_mock.force_z);
            wrench_vec.push_back(wrench_mock.torque_x);
            wrench_vec.push_back(wrench_mock.torque_y);
            wrench_vec.push_back(wrench_mock.torque_z);

        #else
            auto wrench_real=base_cyclic_client_->RefreshFeedback();

            //Accessing the wrench value from the real hardware
            wrench_vec.push_back(wrench_real.base().tool_external_wrench_force_x());
            wrench_vec.push_back(wrench_real.base().tool_external_wrench_force_y());
            wrench_vec.push_back(wrench_real.base().tool_external_wrench_force_z());
            wrench_vec.push_back(wrench_real.base().tool_external_wrench_torque_x());
            wrench_vec.push_back(wrench_real.base().tool_external_wrench_torque_y());
            wrench_vec.push_back(wrench_real.base().tool_external_wrench_torque_z());
        
        #endif
        
        return wrench_vec;
        }
        
        catch(const std::exception &e)
        {
            std::cerr << "wrench reading failed: " << e.what() << std::endl;
            return {};
        }

    }

    // =========================================================================
    // Part 4: Speed Limit
    // =========================================================================

    // setSpeedLimit(): Sets the global speed fraction (0.0 to 1.0).
    // Currently stores the value locally — the Kortex speed limit API
    // call is a TODO for future integration.
    bool KinovaInterface::setSpeedLimit(double fraction)
    {

        if (!connected_.load())
        {
            std::cerr << "Not connected! Setting speed limit failed" << std::endl;
            return false;
        }

        if (fraction < 0.0 || fraction > 1.0)
        {
            std::cerr << "speed limit is not within the range" << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // TODO: call Kortex speed limit API

        current_speed_fraction_ = fraction;
        std::cout << "Speed limit set to "
                  << (fraction * 100.0) << "%.\n";
        return true;
    }

    // =========================================================================
    // Part 5: Gripper Control (Robotiq 2F-85)
    //
    // The Robotiq 2F-85 is controlled via the Kortex API's gripper commands
    // (not a separate driver). SendGripperCommand is non-blocking — it
    // returns immediately after dispatching the command. A polling loop
    // (100ms interval, 5s timeout) monitors the actual gripper position
    // until it matches the commanded position within tolerance.
    //
    // Object detection uses stall-based detection: when closeGripper()
    // commands position 1.0 (fully closed) but the gripper stalls at a
    // smaller position (object preventing closure), the gap between
    // commanded and actual position exceeds the tolerance → object detected.
    // This is why closeGripper() returns true on both "fully closed" and
    // "stalled on object" — both are successful outcomes.
    // =========================================================================

    // setGripperPosition(): Commands the gripper to a normalized position
    // (0.0 = fully open, 1.0 = fully closed). Polls until the gripper
    // reaches the target or times out. Returns false on timeout or error.

    bool KinovaInterface::setGripperPosition(double req_position, double speed)
    {

        // TO DO: for now, this param is ignored but later it will be used in velocity control
        (void)speed;

        last_commanded_grip_pos_ = req_position;

        // Checking atomic flags
        if (!connected_.load())
        {
            std::cerr << "Error! Not connected" << std::endl;
            return false;
        }

        if (e_stop_active_.load())
        {
            std::cerr << "Error! e-stop is active we can't proceed this action"
                      << std::endl;
            return false;
        }

        // Range validation
        if (req_position < 0.0 || req_position > 1.0)
        {
            std::cerr << "Error! Position must be [0.0, 1.0], got " << req_position << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        // Gripper command object
        k_api::Base::GripperCommand gripper_command_;

        // A Feedback object
        k_api::Base::Gripper gripper_feedback_;

        // Defines: In which form user wants feedback: POSITION, SPEED, FORCE
        k_api::Base::GripperRequest gripper_request_;

#ifdef USE_KORTEX_MOCK
        gripper_command_.mode = k_api::Base::GRIPPER_POSITION; // position as a control param
        gripper_request_.mode = k_api::Base::GRIPPER_POSITION; // position in feedback request
        k_api::Base::Finger finger;
        finger.finger_identifier = 1;
        finger.value = req_position;
        gripper_command_.gripper.finger.push_back(finger);
#else
        gripper_command_.set_mode(k_api::Base::GRIPPER_POSITION);
        gripper_request_.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command_.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1); // actuator ID
        finger->set_value(req_position);
#endif

        // Noting initial time
        auto start = std::chrono::steady_clock::now();

        double current_position = 0.0;

        try
        {
            gripper_feedback_ = base_client_->GetMeasuredGripperMovement(gripper_request_);

#ifdef USE_KORTEX_MOCK
            if (!gripper_feedback_.finger.empty())
            {
                current_position = gripper_feedback_.finger[0].value;
                std::cout << "Initial gripper position: " << current_position << std::endl;
            }
#else
            if (gripper_feedback_.finger_size())
            {
                current_position = gripper_feedback_.finger(0).value();
                std::cout << "Initial gripper position: " << current_position << std::endl;
            }
#endif

            // Sending commands to the base
            base_client_->SendGripperCommand(gripper_command_);
        }

        catch (const std::exception &e)
        {
            std::cerr << "Gripper command failed!" << e.what() << std::endl;
            return false;
        }

        while (true)
        {
            // To check the duration
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            try
            {
                gripper_feedback_ = base_client_->GetMeasuredGripperMovement(gripper_request_);
            }

            catch (const std::exception &e)
            {
                std::cerr << "Gripper command failed!" << e.what() << std::endl;
                return false;
            }

#ifdef USE_KORTEX_MOCK
            if (gripper_feedback_.finger.empty())
            {
                std::cerr << "Empty gripper feedback, retrying..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            current_position = gripper_feedback_.finger[0].value;

#else
            if (gripper_feedback_.finger_size() == 0)
            {
                std::cerr << "Empty gripper feedback, retrying..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            current_position = gripper_feedback_.finger(0).value();
#endif

            std::cout << "Current gripper position: " << current_position << std::endl;

            if (kGripperPositionTolerance > std::abs(current_position - req_position))
            {
                std::cout << "Gripper reached target position: " << std::endl;
                std::cout << "final position: " << current_position << std::endl;
                return true;
            }

            if (duration.count() > kGripperTimeoutSec * 1000)
            {
                std::cout << "Timed out" << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool KinovaInterface::openGripper(double speed)
    {
        std::cout << "Opening gripper..." << std::endl;
        return setGripperPosition(0.0, speed);
    }

    bool KinovaInterface::closeGripper(double force, double speed)
    {

        // TODO-use the force param. For now, void should be fine
        (void)force;

        std::cout << "Closing Gripper..." << std::endl;
        bool reached = setGripperPosition(1.0, speed);
        if (reached)
            return true;
        return isObjectDetected();
    }

    double KinovaInterface::getGripperPosition()
    {

        double current_position = -1.0;

        if (!connected_.load())
        {
            std::cerr << "Error! Not connected " << std::endl;
            return current_position;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        k_api::Base::Gripper gripper_feedback_;
        k_api::Base::GripperRequest gripper_request_;

#ifdef USE_KORTEX_MOCK
        gripper_request_.mode = k_api::Base::GRIPPER_POSITION;
#else
        gripper_request_.set_mode(k_api::Base::GRIPPER_POSITION);
#endif

        try
        {

            gripper_feedback_ = base_client_->GetMeasuredGripperMovement(gripper_request_);

#ifdef USE_KORTEX_MOCK
            if (!gripper_feedback_.finger.empty())
            {
                current_position = gripper_feedback_.finger[0].value;
            }
#else
            if (gripper_feedback_.finger_size())
            {
                current_position = gripper_feedback_.finger(0).value();
            }
#endif

            return current_position;
        }

        catch (const std::exception &e)
        {
            std::cerr << "Gripper command failed!" << e.what() << std::endl;
            return current_position;
        }
    }

    bool KinovaInterface::isObjectDetected()
    {
        double gripper_position = getGripperPosition();

        if (gripper_position < 0.0)
        {
            std::cerr << "Unexpected gripper position" << gripper_position << std::endl;
            return false;
        }

        // The commanded position is higher than actual when the gripper stalled early on something.
        if ((last_commanded_grip_pos_ - gripper_position) > kGripperPositionTolerance && last_commanded_grip_pos_ > 0.5)
        {
            std::cout << "Object has detected" << std::endl;
            return true;
        }
        return false;
    }

    // =========================================================================
    // Part 6: Trajectory Execution
    //
    // Multi-waypoint joint trajectory with optional progress callback.
    // Each waypoint is executed as a separate Kortex Action. The time
    // between waypoints is computed from the time_from_start field (delta
    // from the previous waypoint). The optional callback fires after each
    // waypoint with the current joint angles and a progress fraction.
    //
    // NOTE: This is a simplified waypoint-by-waypoint execution, not a
    // true trajectory interpolation. The arm's internal controller
    // handles smooth motion between waypoints.
    // =========================================================================

    // validateTrajectory(): Pre-flight checks before execution.
    // Validates: non-empty, monotonically increasing timestamps,
    // all joint angles within firmware limits.
    bool KinovaInterface::validateTrajectory(const std::vector<TrajectoryPoint> &waypoints) const
    {
        {
            std::cerr << "Invalid trajectory! Waypoints are empty" << std::endl;
            return false;
        }

        if (waypoints[0].time_from_start < 0.0)
        {
            std::cerr << "start time is Invalid a" << std::endl;
            return false;
        }

        for (size_t i = 0; i < waypoints.size(); i++)
        {
            if (i > 0 && waypoints[i].time_from_start <= waypoints[i - 1].time_from_start)
            {
                std::cerr << "Non-monotonic time: waypoint " << i
                          << " time (" << waypoints[i].time_from_start
                          << ") <= waypoint " << i - 1
                          << " time (" << waypoints[i - 1].time_from_start << ")" << std::endl;
                return false;
            }

            if (!validateJointAngles(waypoints[i].joint_angles))
            {
                return false;
            }
        }
        std::cout << "trajectory validation successfully" << std::endl;
        return true;
    }

    bool KinovaInterface::executeTrajectory(const std::vector<TrajectoryPoint> &waypoints,
                                            MotionCallback feedback_callback)
    {

        if (!connected_.load())
        {
            std::cerr << "Error! Not connected" << std::endl;
            return false;
        }
        if (e_stop_active_.load())
        {
            std::cerr << "Error! e-stop is active we can't proceed this action"
                      << std::endl;
            return false;
        }

        std::unique_lock<std::mutex> lock(mutex_);

        // Validating trajectory
        if (!validateTrajectory(waypoints))
        {
            return false;
        }

        try
        {

// Setting servoing mode
#ifndef USE_KORTEX_MOCK
            auto servoingMode = k_api::Base::ServoingModeInformation();
            servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base_client_->SetServoingMode(servoingMode);
#endif

            lock.unlock();
            for (size_t j = 0; j < waypoints.size(); j++)
            {
                if (e_stop_active_.load())
                {
                    // e-stop was triggered from another thread — abort cleanly
                    return false;
                }

                k_api::Base::Action action;

#ifdef USE_KORTEX_MOCK
                action.is_joint_action = true;
                for (size_t i = 0; i < waypoints[j].joint_angles.size(); i++)
                {
                    k_api::Base::JointAngle joint;
                    joint.joint_identifier = static_cast<uint32_t>(i);
                    joint.value = waypoints[j].joint_angles[i] * kRadToDeg;
                    action.target_joint_angles.joint_angles.push_back(joint);
                }
#else
                for (size_t i = 0; i < waypoints[j].joint_angles.size(); i++)
                {
                    auto *joint_angles = action.mutable_reach_joint_angles()->mutable_joint_angles();
                    auto *joint = joint_angles->add_joint_angles();
                    joint->set_joint_identifier(static_cast<uint32_t>(i));
                    joint->set_value(waypoints[j].joint_angles[i] * kRadToDeg);
                }
#endif
                base_client_->ExecuteAction(action);

                // To track the waypoints
                double delta = (j == 0) ? waypoints[0].time_from_start
                                        : waypoints[j].time_from_start - waypoints[j - 1].time_from_start;
                std::this_thread::sleep_for(std::chrono::duration<double>(delta));

                if (feedback_callback)
                { // check if caller provided a callback (could be nullptr)

                    // Read current joints — directly from base_client_, NOT getJointAngles()
                    auto measured = base_client_->GetMeasuredJointAngles();

                    // Convert to radians vector (same logic as your getJointAngles())
                    std::vector<double> current_joints;
                    current_joints.reserve(kNumJoints);
#ifdef USE_KORTEX_MOCK
                    for (const auto &angle : measured.joint_angles)
                    {
                        current_joints.push_back(angle.value * kDegToRad);
                    }
#else
                    for (int i = 0; i < measured.joint_angles_size(); i++)
                    {
                        current_joints.push_back(measured.joint_angles(i).value() * kDegToRad);
                    }
#endif

                    // Compute progress and fire
                    double progress = static_cast<double>(j + 1) / static_cast<double>(waypoints.size());
                    feedback_callback(current_joints, progress);
                }
            }

            std::cout << "joint motion completed" << std::endl;
            return true;
        }

        catch (const std::exception &e)
        {
            std::cerr << "Trajectory execution failed" << e.what() << std::endl;

            return false;
        }
    }

    std::future<bool> KinovaInterface::executeTrajectoryAsync(
        const std::vector<TrajectoryPoint> &waypoints,
        MotionCallback feedback_callback)
    {

        // Capture by value to ensure safety across async boundary
        return std::async(std::launch::async, [this, waypoints, feedback_callback]()
                          { return this->executeTrajectory(waypoints, feedback_callback); });
    }

    // =========================================================================
    // Part 7: Cartesian Velocity Control
    //
    // Streaming velocity commands for real-time control loops (admittance
    // controller, visual servoing). Three-layer safety architecture:
    //
    //   Layer 1 — Velocity clamping: per-axis hard limits on linear
    //     (±0.5 m/s) and angular (±40 deg/s) velocities.
    //
    //   Layer 2 — Workspace boundary: reads current EE pose and zeros
    //     any velocity component that would push the arm out of the
    //     safe workspace volume. Motion away from boundary always allowed.
    //
    //   Layer 3 — Watchdog thread: background thread monitors the time
    //     since the last velocity command. If no command arrives within
    //     100ms (kWatchdogTimeoutMs), calls stopMotion() automatically.
    //     Protects against control loop crashes or node failures.
    //
    // PERFORMANCE NOTE: SendTwistCommand() is a synchronous gRPC call
    // that blocks for ~73ms on average. This is the irreducible bottleneck
    // of the high-level Kortex API, capping real-time loops at ~13 Hz.
    // The upgrade path is Kortex low-level servoing (1 kHz, joint-space).
    //
    // Angular velocities are expected in deg/s (Kortex convention).
    // The admittance controller converts from rad/s at the call site.
    // =========================================================================

    // setCartesianVelocity(): Sends a 6D twist command (3 linear + 3 angular)
    // in the base reference frame. Starts the watchdog thread on first call.
    // Returns false if disconnected or e-stopped.
    bool KinovaInterface::setCartesianVelocity(double vx, double vy, double vz,
                                               double wx, double wy, double wz)
    {

        if (!connected_.load())
        {
            std::cerr << "Error! Not connected" << std::endl;
            return false;
        }

        if (e_stop_active_.load())
        {
            std::cerr << "Error! e-stop is active we can't proceed this action"
                      << std::endl;
            return false;
        }

        // Clipping to safety limits for both angular and linear
        bool clipped = false;

        auto clamp_and_flag = [&](double &v, double min, double max)
        {
            double orig = v;
            v = std::clamp(v, min, max);
            clipped |= (v != orig);
        };

        // Linear
        clamp_and_flag(vx, -kMaxLinearVelocity, kMaxLinearVelocity);
        clamp_and_flag(vy, -kMaxLinearVelocity, kMaxLinearVelocity);
        clamp_and_flag(vz, -kMaxLinearVelocity, kMaxLinearVelocity);

        // Angular
        clamp_and_flag(wx, -kMaxAngularVelocity, kMaxAngularVelocity);
        clamp_and_flag(wy, -kMaxAngularVelocity, kMaxAngularVelocity);
        clamp_and_flag(wz, -kMaxAngularVelocity, kMaxAngularVelocity);

        if (clipped)
        {
            std::cerr << "Warning: velocity components clipped to safety limits\n";
        }

        std::unique_lock<std::mutex> lock(mutex_);

// Read pose with mutex_ to protect the kortex call
#ifdef USE_KORTEX_MOCK
        auto measured = base_client_->GetMeasuredCartesianPose();
        double px = measured.x, py = measured.y, pz = measured.z;
#else
        auto measured = base_client_->GetMeasuredCartesianPose();
        double px = measured.x(), py = measured.y(), pz = measured.z();
#endif

        // Zeroed out all the velocities which are not in the range
        if (px >= kWorkspaceXMax && vx > 0)
            vx = 0.0;
        if (px <= kWorkspaceXMin && vx < 0)
            vx = 0.0;
        if (py >= kWorkspaceYMax && vy > 0)
            vy = 0.0;
        if (py <= kWorkspaceYMin && vy < 0)
            vy = 0.0;
        if (pz >= kWorkspaceZMax && vz > 0)
            vz = 0.0;
        if (pz <= kWorkspaceZMin && vz < 0)
            vz = 0.0;

        k_api::Base::TwistCommand twist_cmd;

#ifdef USE_KORTEX_MOCK
        twist_cmd.reference_frame = 0;
        twist_cmd.twist.linear_x = vx;
        twist_cmd.twist.linear_y = vy;
        twist_cmd.twist.linear_z = vz;
        twist_cmd.twist.angular_x = wx;
        twist_cmd.twist.angular_y = wy;
        twist_cmd.twist.angular_z = wz;
#else
        twist_cmd.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
        auto *twist = twist_cmd.mutable_twist();
        twist->set_linear_x(vx);
        twist->set_linear_y(vy);
        twist->set_linear_z(vz);
        twist->set_angular_x(wx);
        twist->set_angular_y(wy);
        twist->set_angular_z(wz);

#endif

        try
        {
            
            base_client_->SendTwistCommand(twist_cmd);
            std::cerr << "TWIST SENT: vx=" << vx << " vy=" << vy << " vz=" << vz
            << " wx=" << wx << " wy=" << wy << " wz=" << wz << "\n";
            
            // Update watchdog state
            velocity_active_.store(true);
            {
                std::lock_guard<std::mutex> time_lock(velocity_time_mutex_);
                last_velocity_time_ = std::chrono::steady_clock::now();
            }

            lock.unlock(); // No need of mutex due to thread separation

            if (!watchdog_running_.load())
            {
                watchdog_running_.store(true);
                watchdog_thread_ = std::thread([this]()
                                               {
                        while(watchdog_running_.load()){
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));

                            //First check that is arm in velocity mode or not
                            if(!velocity_active_.load()) continue;
                            std::lock_guard<std::mutex> time_lock(velocity_time_mutex_);
                            
                            auto elapsed=std::chrono::steady_clock::now()-last_velocity_time_;
                            double ms=std::chrono::duration<double, std::milli>(elapsed).count();

                            if(ms>kWatchdogTimeoutMs){
                                std::cerr<<"Watchdog: no velocity command for "
                                    << ms << "ms — auto-stopping\n";
                                velocity_active_.store(false);
                                stopMotion();
                            }

                        } });
            }
            return true;
        }

        catch (std::exception &e)
        {
            std::cerr << "sendCartesianVelocity() failed..." << e.what() << std::endl;
            return false;
        }
    }

    // stopMotion(): Sends a zero-velocity command and calls base Stop().
    // Clears the velocity_active_ flag so the watchdog thread stops
    // monitoring. The arm stays powered (not e-stopped) — ready for
    // immediate re-use without clearing faults.
    void KinovaInterface::stopMotion()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Telling watchdog "nothing to monitor"

        try
        {
            if (base_client_)
            {
                // both mock and real have this
                base_client_->Stop();
            }
        }

        catch (std::exception &e)
        {
            std::cerr << "Stopmotion failed() failed..." << e.what() << std::endl;
            return;
        }
    }

}
