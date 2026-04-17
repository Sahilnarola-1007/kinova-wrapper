#include "kinova_interface/KinovaInterface.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <thread>
#include <chrono>

int main() {
    using namespace kinova_wrapper;

    // Instantiate Kinova interface object for robot communication
    KinovaInterface kinova;
    bool ok;

    // ============================================================
    // STEP 1: Establish connection with the robot
    // ============================================================
    ok = kinova.connect("192.168.1.10");
    std::cout << "Connect(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    // Allow time for stable connection setup
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // ============================================================
    // STEP 2: Retrieve and display current gripper position
    // ============================================================
    std::cout << "Current gripper position := " 
              << kinova.getGripperPosition() << std::endl;
    std::cout << "====================" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ============================================================
    // STEP 3: Open gripper and verify position
    // ============================================================
    ok = kinova.openGripper();
    std::cout << "Open gripper position := " 
              << kinova.getGripperPosition() << std::endl;
    std::cout << "openGripper(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ============================================================
    // STEP 4: Set gripper to half-open position (0.5)
    // and validate the commanded position
    // ============================================================
    ok = kinova.setGripperPosition(0.5); // Half-open command
    std::cout << "Commanded position: 0.5 | Actual position := "
              << kinova.getGripperPosition() << std::endl;
    std::cout << "setGripperPosition(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ============================================================
    // STEP 5: Close gripper (no object present)
    // Verify position and ensure no object is detected
    // ============================================================
    ok = kinova.closeGripper();
    std::cout << "Close gripper position := " 
              << kinova.getGripperPosition() << std::endl;
    std::cout << "closeGripper(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));
 
    // ============================================================
    // STEP 6: Open gripper and prompt user to insert object
    // Then close gripper and verify object detection
    // ============================================================
    ok = kinova.openGripper();
    std::cout << "openGripper(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;

    std::cout << "Place an object between the gripper fingers, then press Enter..." << std::endl;
    std::cin.get();
    
    ok = kinova.closeGripper();
    std::cout << "closeGripper(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;

    std::cout << "Closed gripper position with object := "
              << kinova.getGripperPosition() << std::endl;

    // Check if object is detected (expected: true)
    bool status = kinova.isObjectDetected();
    std::cout << "Is an object detected?: " << status << std::endl;
    std::cout << "====================" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ============================================================
    // STEP 7: Open gripper to release object
    // ============================================================
    ok = kinova.openGripper();
    std::cout << "openGripper(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    std::cout << "====================" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // ============================================================
    // STEP 8: Disconnect from robot
    // ============================================================
    kinova.disconnect();
    std::cout << "Disconnected from robot." << std::endl;
    std::cout << "====================" << std::endl;

    return 0;
}
