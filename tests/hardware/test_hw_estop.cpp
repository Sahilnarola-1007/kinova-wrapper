#include "kinova_interface/KinovaInterface.hpp"
#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>

int main() {
    using namespace kinova_wrapper;

    // Instantiate Kinova interface for robot communication
    KinovaInterface kinova;

    // ============================================================
    // STEP 1: Connect to robot and configure motion parameters
    // ============================================================
    kinova.connect("192.168.1.10");
    kinova.setSpeedLimit(0.1);  // Limit joint speed to 10% for safety

    // ============================================================
    // STEP 2: Read and display current joint angles
    // ============================================================
    auto current = kinova.getJointAngles();
    auto original = current;  // Store original configuration for later recovery

    std::cout << "Current joint angles:" << std::endl;
    for (size_t i = 0; i < current.size(); i++) {
        std::cout << " joint " << i << " := " << current[i] << std::endl;
    }
    std::cout << "====================" << std::endl;

    // ============================================================
    // STEP 3: Create a target configuration
    // Modify only joint 0 by +1.57 radians (90 degrees)
    // ============================================================
    std::vector<double> target = current;
    target[0] += 1.57;  // Increment joint 0

    // ============================================================
    // STEP 4: Start asynchronous motion toward target
    // ============================================================
    auto motion_future = kinova.moveToJointAnglesAsync(target);

    // Allow robot to begin motion before triggering emergency stop
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ============================================================
    // STEP 5: Trigger emergency stop during motion
    // ============================================================
    kinova.emergencyStop();

    bool motion_result = motion_future.get();
    std::cout << "Original motion after E-Stop: " << (motion_result ? "COMPLETED" : "ABORTED") << std::endl;

    bool estop = kinova.isEStopActive();
    std::cout << "isEStopActive(): " << (estop ? "TRUE" : "FALSE") << std::endl;
    std::cout << "====================" << std::endl;

    // ============================================================
    // STEP 6: Attempt motion while E-Stop is active
    // Expected: motion should fail
    // ============================================================
    std::future<bool> ok = kinova.moveToJointAnglesAsync(target);
    std::cout << "moveToJointAnglesAsync() during E-Stop: "
              << (ok.get() ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    // ============================================================
    // STEP 7: Clear emergency stop condition
    // ============================================================
    bool clear = kinova.clearEmergencyStop();
    std::cout << "clearEmergencyStop(): " << (clear ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    // ============================================================
    // STEP 8: Read and display joint angles after interruption
    // Robot should have stopped mid-motion
    // ============================================================
    current = kinova.getJointAngles();
    std::cout << "Joint angles after E-Stop:" << std::endl;
    for (size_t i = 0; i < 7; i++) {
        std::cout << " joint " << i << " := " << current[i] << std::endl;
    }
    std::cout << "====================" << std::endl;

    // ============================================================
    // STEP 9: Return robot to original configuration
    // ============================================================
    auto future = kinova.moveToJointAnglesAsync(original);
    auto status = future.get();
    assert(status);
    std::cout << "Returned to original configuration" << std::endl;

    // ============================================================
    // STEP 10: Disconnect from robot
    // ============================================================
    kinova.disconnect();
    std::cout << "Disconnected from robot." << std::endl;
    std::cout << "====================" << std::endl;

    return 0;
}
