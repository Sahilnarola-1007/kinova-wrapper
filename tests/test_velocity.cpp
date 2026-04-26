#include<gtest/gtest.h>
#include"kinova_wrapper/KinovaInterface.hpp"

using namespace kinova_wrapper;
 
TEST(VelocityTest, VelocityRejectedWhenDisconnected) {
    kinova_wrapper::KinovaInterface kinova;
    bool result = kinova.setCartesianVelocity(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

    EXPECT_FALSE(result);
}

TEST(VelocityTest, VelocityRejectedWhenEstop) {
    kinova_wrapper::KinovaInterface kinova;

    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    kinova.emergencyStop();
    bool result = kinova.setCartesianVelocity(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

    EXPECT_FALSE(result);
}

TEST(VelocityTest, VelocityClipping) {
    kinova_wrapper::KinovaInterface kinova;

    ASSERT_TRUE(kinova.connect("192.168.1.10"));
    kinova.setCartesianVelocity(999, 0.0, 0.0, 0.0, 0.0, 0.0);

    auto* base = kinova.getBaseClientForTesting();
    auto twist = base->GetLastTwist();

    EXPECT_DOUBLE_EQ(twist.twist.linear_x, 0.5);  ///999 clipped to 0.5
}

TEST(VelocityTest, StopMotionZerosVelocity) {
    kinova_wrapper::KinovaInterface kinova;

    ASSERT_TRUE(kinova.connect("192.168.1.10"));
    kinova.setCartesianVelocity(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
    kinova.stopMotion();

    auto* base = kinova.getBaseClientForTesting();
    auto twist = base->GetLastTwist();
    EXPECT_DOUBLE_EQ(twist.twist.linear_x, 0.0);
}

TEST(VelocityTest, WatchdogAutoStop) {
    kinova_wrapper::KinovaInterface kinova;

    ASSERT_TRUE(kinova.connect("192.168.1.10"));
    kinova.setCartesianVelocity(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
    bool result=kinova.isVelocityActive();
    EXPECT_FALSE(result);

}