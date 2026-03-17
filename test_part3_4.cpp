#include "kinova_interface/KinovaInterface.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    using namespace kinova_wrapper;

    KinovaInterface kinova;

    // =========================================================================
    // State reading before connect — should return empty/zeroed
    // =========================================================================
    std::cout << "=== Test 1: getJointAngles before connect ===\n";
    auto angles = kinova.getJointAngles();
    assert(angles.empty());
    std::cout << "PASS: Empty vector returned.\n\n";

    std::cout << "=== Test 2: getCurrentPose before connect ===\n";
    Pose pose = kinova.getCurrentPose();
    assert(pose.x == 0.0 && pose.y == 0.0 && pose.z == 0.0);
    std::cout << "PASS: Zeroed pose returned.\n\n";

    std::cout << "=== Test 3: getWrench before connect ===\n";
    auto wrench = kinova.getWrench();
    assert(wrench.empty());
    std::cout << "PASS: Empty vector returned.\n\n";

    // Connect
    kinova.connect("192.168.1.10");

    // =========================================================================
    // State reading after connect
    // =========================================================================
    std::cout << "=== Test 4: getJointAngles after connect ===\n";
    angles = kinova.getJointAngles();
    assert(angles.size() == 7);
    std::cout << "PASS: Got 7 joint angles.\n";
    std::cout << "  Values: ";
    for (double a : angles) { std::cout << a << " "; }
    std::cout << "\n\n";

    std::cout << "=== Test 5: getCurrentPose after connect ===\n";
    pose = kinova.getCurrentPose();
    // Mock returns zeroed pose, so just verify it returns without crashing
    std::cout << "PASS: Got pose (x=" << pose.x << ", y=" << pose.y
              << ", z=" << pose.z << ").\n\n";

    std::cout << "=== Test 6: getWrench after connect ===\n";
    wrench = kinova.getWrench();
    assert(wrench.size() == 6);
    std::cout << "PASS: Got 6 wrench values.\n";
    std::cout << "  Values: ";
    for (double w : wrench) { std::cout << w << " "; }
    std::cout << "\n\n";

    // =========================================================================
    // State reading after motion (verify state updates)
    // =========================================================================
    std::cout << "=== Test 7: Joint angles after motion ===\n";
    kinova.moveToJointAngles({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
    angles = kinova.getJointAngles();
    assert(angles.size() == 7);
    // Mock stores commanded angles (in degrees), we read back (in radians)
    // Check first joint: we sent 0.1 rad → mock stores 5.73 deg → we read back 0.1 rad
    assert(std::abs(angles[0] - 0.1) < 0.01);
    assert(std::abs(angles[1] - 0.2) < 0.01);
    std::cout << "PASS: Joint angles match commanded values.\n\n";

    // =========================================================================
    // Speed limit
    // =========================================================================
    std::cout << "=== Test 8: setSpeedLimit valid (0.5) ===\n";
    bool ok = kinova.setSpeedLimit(0.5);
    assert(ok);
    std::cout << "PASS: Speed limit set to 50%.\n\n";

    std::cout << "=== Test 9: setSpeedLimit boundary (0.0) ===\n";
    ok = kinova.setSpeedLimit(0.0);
    assert(ok);
    std::cout << "PASS: Speed limit 0% accepted.\n\n";

    std::cout << "=== Test 10: setSpeedLimit boundary (1.0) ===\n";
    ok = kinova.setSpeedLimit(1.0);
    assert(ok);
    std::cout << "PASS: Speed limit 100% accepted.\n\n";

    std::cout << "=== Test 11: setSpeedLimit invalid (negative) ===\n";
    ok = kinova.setSpeedLimit(-0.1);
    assert(!ok);
    std::cout << "PASS: Negative speed limit rejected.\n\n";

    std::cout << "=== Test 12: setSpeedLimit invalid (> 1.0) ===\n";
    ok = kinova.setSpeedLimit(1.5);
    assert(!ok);
    std::cout << "PASS: Speed limit > 1.0 rejected.\n\n";

    std::cout << "=== Test 13: setSpeedLimit when not connected ===\n";
    kinova.disconnect();
    ok = kinova.setSpeedLimit(0.5);
    assert(!ok);
    std::cout << "PASS: Speed limit rejected — not connected.\n\n";

    std::cout << "=== ALL PARTS 3 & 4 TESTS PASSED ===\n";
    return 0;
}
