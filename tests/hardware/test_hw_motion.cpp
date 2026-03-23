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

    std::cout << "=== Test 4: Valid joint angles (all zeros) ===\n";
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0});
    assert(ok);
    std::cout << "PASS: Zero angles accepted.\n\n";

    std::cout << "=== Test 5: Valid angles within limits ===\n";
    // Joint 2 limit is ±128.9 deg. 1.0 rad = 57.3 deg — well within limits.
    ok = kinova.moveToJointAngles({0.5, 1.0, 0.5, -1.0, 0.5, -0.5, 0.5});
    assert(ok);
    std::cout << "PASS: Valid angles accepted.\n\n";

    std::cout << "=== Test 6: Angle exceeds joint limit ===\n";
    // Joint 2 limit is ±128.9 deg. 3.0 rad = 171.9 deg — exceeds limit.
    ok = kinova.moveToJointAngles({0, 3.0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Out-of-limit angle rejected.\n\n";

    // =========================================================================
    // Async motion
    // =========================================================================
    std::cout << "=== Test 7: Async joint motion ===\n";
    auto future = kinova.moveToJointAnglesAsync({0, 0.5, 0, -0.5, 0, 0.5, 0});
    // future.get() blocks until motion completes
    ok = future.get();
    assert(ok);
    std::cout << "PASS: Async joint motion completed.\n\n";

    std::cout << "=== Test 8: Async with invalid angles (should fail) ===\n";
    auto future2 = kinova.moveToJointAnglesAsync({0, 0, 0});  // only 3 angles
    ok = future2.get();
    assert(!ok);
    std::cout << "PASS: Async rejected invalid angles.\n\n";

    // =========================================================================
    // Cartesian motion
    // =========================================================================
    std::cout << "=== Test 9: Valid Cartesian pose ===\n";
    Pose target;
    target.x = 0.5;
    target.y = 0.2;
    target.z = 0.3;
    target.theta_x = 90.0;
    target.theta_y = 0.0;
    target.theta_z = 180.0;
    ok = kinova.moveToCartesianPose(target);
    assert(ok);
    std::cout << "PASS: Cartesian motion completed.\n\n";

    std::cout << "=== Test 10: Async Cartesian pose ===\n";
    Pose target2;
    target2.x = 0.4;
    target2.y = 0.1;
    target2.z = 0.5;
    auto future3 = kinova.moveToCartesianPoseAsync(target2);
    ok = future3.get();
    assert(ok);
    std::cout << "PASS: Async Cartesian motion completed.\n\n";

    // =========================================================================
    // E-stop blocks motion
    // =========================================================================
    std::cout << "=== Test 11: Motion blocked by e-stop ===\n";
    kinova.emergencyStop();
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Motion rejected — e-stop active.\n\n";

    std::cout << "=== Test 12: Cartesian blocked by e-stop ===\n";
    ok = kinova.moveToCartesianPose(target);
    assert(!ok);
    std::cout << "PASS: Cartesian rejected — e-stop active.\n\n";

    // =========================================================================
    // Motion after disconnect
    // =========================================================================
    kinova.clearEmergencyStop();
    kinova.disconnect();
    std::cout << "=== Test 13: Motion after disconnect ===\n";
    ok = kinova.moveToJointAngles({0, 0, 0, 0, 0, 0, 0});
    assert(!ok);
    std::cout << "PASS: Motion rejected — disconnected.\n\n";

    std::cout << "=== ALL PART 2 TESTS PASSED ===\n";
    return 0;
}
