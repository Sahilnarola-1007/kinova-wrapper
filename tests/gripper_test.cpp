#include <gtest/gtest.h>
#include "kinova_interface/KinovaInterface.hpp"

// =============================================================================
// GripperTest — Tests for Robotiq 2F-85 gripper control via KinovaInterface
// =============================================================================
//
// All tests use the mock SDK (compiled with -DUSE_KORTEX_MOCK).
// Real hardware validation is separate.
// =============================================================================

class GripperTest : public ::testing::Test {
protected:
    kinova_wrapper::KinovaInterface kinova;

    void SetUp() override {
        // Most tests need a connected robot
        // Individual tests that test disconnected behavior skip this
    }

    void connectRobot() {
        ASSERT_TRUE(kinova.connect("192.168.1.10", 10000, "admin", "admin"));
    }
};

// =============================================================================
// setGripperPosition — Valid inputs
// =============================================================================

TEST_F(GripperTest, SetGripperPosition_Valid_Open) {
    connectRobot();
    EXPECT_TRUE(kinova.setGripperPosition(0.0));
}

TEST_F(GripperTest, SetGripperPosition_Valid_Half) {
    connectRobot();
    EXPECT_TRUE(kinova.setGripperPosition(0.5));
}

TEST_F(GripperTest, SetGripperPosition_Valid_Closed) {
    connectRobot();
    EXPECT_TRUE(kinova.setGripperPosition(1.0));
}

// =============================================================================
// setGripperPosition — Invalid inputs
// =============================================================================

TEST_F(GripperTest, SetGripperPosition_Invalid_Negative) {
    connectRobot();
    EXPECT_FALSE(kinova.setGripperPosition(-0.1));
}

TEST_F(GripperTest, SetGripperPosition_Invalid_Over) {
    connectRobot();
    EXPECT_FALSE(kinova.setGripperPosition(1.5));
}

// =============================================================================
// setGripperPosition — Not connected
// =============================================================================

TEST_F(GripperTest, SetGripperPosition_NotConnected) {
    // Do NOT call connectRobot()
    EXPECT_FALSE(kinova.setGripperPosition(0.5));
}

// =============================================================================
// openGripper / closeGripper
// =============================================================================

TEST_F(GripperTest, OpenGripper_Connected) {
    connectRobot();
    EXPECT_TRUE(kinova.openGripper());
}

TEST_F(GripperTest, CloseGripper_Connected) {
    connectRobot();
    EXPECT_TRUE(kinova.closeGripper());
}

// =============================================================================
// getGripperPosition
// =============================================================================

TEST_F(GripperTest, GetGripperPosition_Connected) {
    connectRobot();
    kinova.setGripperPosition(0.7);
    double pos = kinova.getGripperPosition();
    EXPECT_GE(pos, 0.0);
    EXPECT_LE(pos, 1.0);
    EXPECT_NEAR(pos, 0.7, 0.01);
}

TEST_F(GripperTest, GetGripperPosition_NotConnected) {
    // Do NOT call connectRobot()
    double pos = kinova.getGripperPosition();
    EXPECT_DOUBLE_EQ(pos, -1.0);
}

// =============================================================================
// isObjectDetected
// =============================================================================

TEST_F(GripperTest, IsObjectDetected_WithObject) {
    connectRobot();

    // Enable stall simulation — gripper will report 0.4 instead of 1.0
    auto* mock_base = kinova.getBaseClientForTesting();
    mock_base->setSimulateObject(true);

    // Close gripper — commands 1.0 but mock returns 0.4 (stalled)
    kinova.closeGripper();

    EXPECT_TRUE(kinova.isObjectDetected());
}

TEST_F(GripperTest, IsObjectDetected_NoObject) {
    connectRobot();

    // Disable stall simulation — gripper reaches commanded position
    auto* mock_base = kinova.getBaseClientForTesting();
    mock_base->setSimulateObject(false);

    // Close gripper — commands 1.0 and mock returns 1.0 (fully closed)
    kinova.closeGripper();

    EXPECT_FALSE(kinova.isObjectDetected());
}

// =============================================================================
// Edge cases
// =============================================================================

TEST_F(GripperTest, SetGripperPosition_EStopActive) {
    connectRobot();
    kinova.emergencyStop();
    EXPECT_FALSE(kinova.setGripperPosition(0.5));
    kinova.clearEmergencyStop();
}

TEST_F(GripperTest, OpenGripper_AfterClose) {
    connectRobot();
    EXPECT_TRUE(kinova.closeGripper());
    EXPECT_TRUE(kinova.openGripper());
    double pos = kinova.getGripperPosition();
    EXPECT_NEAR(pos, 0.0, 0.01);
}
