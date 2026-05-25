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

    KinovaInterface::KinovaInterface()
    {
        std::cout << "KinovaInterface created. Call connect method \
        to build TCP connection"
                  << std::endl;
    };

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

    // =============================================================================
    // Part 1: Connection and disconnection set up
    // =============================================================================

    // fun 1: connect()
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

    // fun 2: disconnectLocked()
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

    // fun 3: disconnect()
    void KinovaInterface::disconnect()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        disconnectLocked();
    }

    // fun 4: isConnected()
    bool KinovaInterface::isConnected() const
    {
        return connected_.load();
    }

    // fun 5: isEStopActive()
    bool KinovaInterface::isEStopActive() const
    {
        return e_stop_active_.load();
    }

    //fun 5: 
    bool KinovaInterface::isVelocityActive() const
    { 
        return velocity_active_.load();
    }
    // =============================================================================
    // Part 2: Motion Commands
    // =============================================================================

    // fun1: validateJointAngles
    // Called under mutex_ — validates angles are within firmware limits (degrees internally)
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

    // fun 2:moveToJointAngles (sync version): Blocking method
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

    // fun 3: moveTojointAnglesAysnc()
    std::future<bool> KinovaInterface::moveToJointAnglesAsync(
        const std::vector<double> &angles)
    {

        // Capture angles BY VALUE into the lambda.
        // Why? The caller's vector might go out of scope before the async thread runs.
        // Capturing by reference would be a dangling reference → undefined behavior.

        return std::async(std::launch::async, [this, angles]()
                          { return this->moveToJointAngles(angles); });
    }

    // fun 4: moveToCartesianPose — mutable_reach_pose()
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

    // fun 5:moveToCartesianPose(Async version)
    std::future<bool> KinovaInterface::moveToCartesianPoseAsync(const Pose &pose)
    {
        return std::async(std::launch::async,
                          [this, pose]()
                          {
                              return this->moveToCartesianPose(pose);
                          });
    }

    // fun 6: emergency stop
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

    // fun 7: Clear emergency stop
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

    // =============================================================================
    // Part 3: state reading
    // =============================================================================

    // fun 1: Reading current joint angles
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

    // fun 2: Reading current pose
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

    // fun 3: Reading wrench:(fx,fy,fz,tx,ty,tz)
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

    // =============================================================================
    // Part 4: setSpeedLimit
    // =============================================================================

    // fun 1: setting speed limit by using fraction
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

    // =============================================================================
    // Part 5: Gripper Control
    // =============================================================================

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

    bool KinovaInterface::validateTrajectory(const std::vector<TrajectoryPoint> &waypoints) const
    {

        if (waypoints.empty())
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

        // Clliping to avoid Overshooting for both angular and linear
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
            std::cerr << "TWIST SENT: vx=" << vx << " vy=" << vy << " vz=" << vz << "\n";

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

    void KinovaInterface::stopMotion()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Telling watchdog "nothing to monitor"
        velocity_active_.store(false);

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
