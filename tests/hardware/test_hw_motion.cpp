#include "kinova_interface/KinovaInterface.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    using namespace kinova_wrapper;

    KinovaInterface kinova;

    // =========================================================================
    // Pre-connection tests — all motion should fail
    // =========================================================================
    std::cout << "=== Test 1: Motion before connect (should fail) ===\n";
    bool ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Motion rejected — not connected.\n\n";

    // Connect for remaining tests
    kinova.connect("192.168.1.10");

    // =========================================================================
    // validateJointAngles (tested indirectly through moveToJointAngles)
    // =========================================================================
    std::cout << "=== Test 2: Wrong number of joints (6 instead of 7) ===\n";
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Rejected — wrong number of joints.\n\n";

    std::cout << "=== Test 3: Wrong number of joints (8 instead of 7) ===\n";
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Rejected — too many joints.\n\n";

    
    std::cout << "=== Test 4: Angle exceeds joint limit ===\n";
    // Joint 2 limit is ±128.9 deg. 3.0 rad = 171.9 deg — exceeds limit.
    ok = kinova.moveToJointAngles({0, 3.0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Out-of-limit angle rejected.\n\n";

    // =========================================================================
    // Async motion
    // =========================================================================
    
    // Read current position once — use it as base for all motion tests
    auto current = kinova.getJointAngles();
    assert(current.size() == 7);

    // Test 5: Async joint motion — small delta on joint 1
    std::cout << "=== Test 5: Async joint motion ===\n";
    std::vector<double> target = current;
    target[0] += 0.174;  // +10 degrees on joint 1 only
    auto future = kinova.moveToJointAnglesAsync(target);
    ok = future.get();
    assert(ok);
    std::cout << "PASS: Async joint motion completed.\n\n";

    // Return to original
    kinova.moveToJointAngles(current);

    // Test 6: invalid angles
    std::cout << "=== Test 6: Async with invalid angles (should fail) ===\n";
    auto future2 = kinova.moveToJointAnglesAsync({0, 0, 0});
    ok = future2.get();
    assert(!ok);
    std::cout << "PASS: Async rejected invalid angles.\n\n";

    // Test 7: Cartesian — small delta from current pose
    std::cout << "=== Test 7: Valid Cartesian pose ===\n";
    Pose current_pose = kinova.getCurrentPose();
    Pose target_pose = current_pose;
    target_pose.x += 0.05;  // move 5cm in x only
    ok = kinova.moveToCartesianPose(target_pose);
    assert(ok);
    std::cout << "PASS: Cartesian motion completed.\n\n";

    // Return
    kinova.moveToCartesianPose(current_pose);

    // Test 8: Async Cartesian — small delta
    std::cout << "=== Test 8: Async Cartesian pose ===\n";
    Pose target2 = current_pose;
    target2.y += 0.05;  // move 5cm in y only
    auto future3 = kinova.moveToCartesianPoseAsync(target2);
    ok = future3.get();
    assert(ok);
    std::cout << "PASS: Async Cartesian motion completed.\n\n";

    // Return
    kinova.moveToCartesianPose(current_pose);
   
    // =========================================================================
    // E-stop blocks motion
    // =========================================================================
    std::cout << "=== Test 9: Motion blocked by e-stop ===\n";
    kinova.emergencyStop();
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Motion rejected — e-stop active.\n\n";

    std::cout << "=== Test 10: Cartesian blocked by e-stop ===\n";
    ok = kinova.moveToCartesianPose(current_pose);
    assert(!ok);
    std::cout << "PASS: Cartesian rejected — e-stop active.\n\n";

    // =========================================================================
    // Motion after disconnect
    // =========================================================================
    kinova.clearEmergencyStop();
    kinova.disconnect();
    std::cout << "=== Test 11: Motion after disconnect ===\n";
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Motion rejected — disconnected.\n\n";

    std::cout << "=== ALL PART 2 TESTS PASSED ===\n";
    return 0;
}
