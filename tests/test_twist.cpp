/**
 * test_twist.cpp — Minimal test: connect, go home, send a small twist.
 * 
 * Build (from ros2_ws):
 *   cd ~/kinova_learning/ros2_ws
 *   colcon build --packages-select kinova_wrapper
 *   g++ -std=c++17 -I install/kinova_wrapper/include \
 *       -I ~/kinova_learning/kortex_api/include \
 *       -I ~/kinova_learning/kortex_api/include/client \
 *       -I ~/kinova_learning/kortex_api/include/client_stubs \
 *       -I ~/kinova_learning/kortex_api/include/common \
 *       -I ~/kinova_learning/kortex_api/include/messages \
 *       -I ~/kinova_learning/kortex_api/include/google \
 *       -D_OS_UNIX \
 *       -o test_twist test_twist.cpp \
 *       -L install/kinova_wrapper/lib -lkinova_wrapper \
 *       -L ~/kinova_learning/kortex_api/lib -lKortexApiCpp -lpthread -ldl \
 *       -Wl,-rpath,install/kinova_wrapper/lib:$HOME/kinova_learning/kortex_api/lib
 * 
 * Run:
 *   ./test_twist
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "kinova_wrapper/KinovaInterface.hpp"

int main() {
    std::cout << "=== Twist Command Test ===" << std::endl;

    // 1. Connect
    std::cout << "[1] Connecting to 192.168.1.10..." << std::endl;
    kinova_wrapper::KinovaInterface kinova;
    if (!kinova.connect("192.168.1.10", 10000, "admin", "admin")) {
        std::cerr << "FAILED to connect!" << std::endl;
        return 1;
    }
    std::cout << "Connected." << std::endl;

    // 2. Print current pose
    auto pose = kinova.getCurrentPose();
    std::cout << "[2] Current pose: x=" << pose.x << " y=" << pose.y 
              << " z=" << pose.z << std::endl;

    // 3. Stop any active command first
    std::cout << "[3] Calling stopMotion()..." << std::endl;
    kinova.stopMotion();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 4. Send a small twist: 0.02 m/s in x for 2 seconds
    std::cout << "[4] Sending twist: vx=0.02 m/s for 2 seconds..." << std::endl;
    std::cout << "    ARM SHOULD MOVE SLOWLY IN +X" << std::endl;
    
    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        double sec = std::chrono::duration<double>(elapsed).count();
        if (sec > 2.0) break;

        bool ok = kinova.setCartesianVelocity(0.02, 0.0, 0.0, 0.0, 0.0, 0.0);
        if (!ok) {
            std::cerr << "setCartesianVelocity returned false!" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 5. Stop
    std::cout << "[5] Stopping..." << std::endl;
    kinova.stopMotion();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 6. Print new pose
    auto pose2 = kinova.getCurrentPose();
    std::cout << "[6] New pose: x=" << pose2.x << " y=" << pose2.y 
              << " z=" << pose2.z << std::endl;
    
    double dx = pose2.x - pose.x;
    double dy = pose2.y - pose.y;
    double dz = pose2.z - pose.z;
    std::cout << "    Delta: dx=" << dx*1000 << "mm dy=" << dy*1000 
              << "mm dz=" << dz*1000 << "mm" << std::endl;

    if (std::abs(dx) > 1.0) {
        std::cout << "    >>> TWIST WORKS! Arm moved " << dx*1000 << "mm in x" << std::endl;
    } else {
        std::cout << "    >>> TWIST FAILED. Arm did not move." << std::endl;
    }

    kinova.disconnect();
    std::cout << "Done." << std::endl;
    return 0;
}
