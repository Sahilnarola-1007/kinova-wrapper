/**
 * @file test_hw_fk_ik.cpp
 * @brief Hardware validation of custom FK/IK against Kortex API on real Kinova Gen3.
 *
 * Tests:
 *   1. FK accuracy: compare computeFK() output vs Kortex GetMeasuredCartesianPose()
 *   2. IK convergence: verify solveIK() reaches the real robot's current pose
 *   3. IK vs Kortex: confirm full chain (IK → FK → tool offset) matches hardware
 *
 * Expected results:
 *   - FK vs Kortex error < 6mm (residual from tool offset approximation + encoder noise)
 *   - IK vs FK error < 0.1mm (internal solver convergence)
 *   - IK vs Kortex error < 6mm (same as FK, since IK converges to FK solution)
 */

#include "kinova_interface/KinovaInterface.hpp"
#include <kinova_kinematics/KinovaKinematics.hpp>
#include <iostream>
#include <cmath>

int main() {
    using namespace kinova_wrapper;

    KinovaKinematics kinematics;
    KinovaInterface kinova;

    // --- Connect to real robot ---
    bool ok = kinova.connect("192.168.1.10");
    std::cout << "Connect(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    if (!ok) return 1;
    std::cout << "====================" << std::endl;

    // --- Read real joint angles (radians) from Kortex ---
    auto joint_angles = kinova.getJointAngles();

    std::cout << "Joint angles (rad): ";
    for (const auto& v : joint_angles) std::cout << v << " ";
    std::cout << std::endl;

    // --- Read Kortex Cartesian pose (measured at Tool Center Point) ---
    auto kortex_pose = kinova.getCurrentPose();
    std::cout << "Kortex TCP position: "
              << kortex_pose.x << " " << kortex_pose.y << " " << kortex_pose.z << std::endl;

    // --- Compute FK using our DH chain + tool offset ---
    std::array<double, 7> q{};
    for (int i = 0; i < 7; i++) q[i] = joint_angles[i];

    auto T = kinematics.computeFK(q);
    auto fk_pos = kinematics.getPosition(T);
    std::cout << "Our FK position:     " << fk_pos.transpose() << std::endl;

    // --- FK vs Kortex: validates DH table + tool offset against real hardware ---
    double dx = kortex_pose.x - fk_pos(0);
    double dy = kortex_pose.y - fk_pos(1);
    double dz = kortex_pose.z - fk_pos(2);
    double fk_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::cout << "FK vs Kortex error:  " << fk_error << " m" << std::endl;
    std::cout << "===============================" << std::endl;

    // --- IK: solve for joint angles that reach the current real pose ---
    std::cout << "IK testing..." << std::endl;

    std::array<double, 7> zero_guess{};
    auto ik_result = kinematics.solveIK(T, zero_guess);
    auto T_check = kinematics.computeFK(ik_result.joint_states);
    auto ik_pos = kinematics.getPosition(T_check);

    // IK vs FK: proves solver convergence (should be < 0.1mm)
    dx = ik_pos(0) - fk_pos(0);
    dy = ik_pos(1) - fk_pos(1);
    dz = ik_pos(2) - fk_pos(2);
    double ik_internal_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::cout << "IK vs FK error:      " << ik_internal_error << " m" << std::endl;

    // IK vs Kortex: validates full chain against real hardware
    dx = ik_pos(0) - kortex_pose.x;
    dy = ik_pos(1) - kortex_pose.y;
    dz = ik_pos(2) - kortex_pose.z;
    double ik_kortex_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::cout << "IK vs Kortex error:  " << ik_kortex_error << " m" << std::endl;
    std::cout << "===============================" << std::endl;

    kinova.disconnect();
    return 0;
}