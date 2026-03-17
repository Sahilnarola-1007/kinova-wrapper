#include<gtest/gtest.h>
#include"kinova_interface/KinovaInterface.hpp"

using namespace kinova_wrapper;
  
// Test 1
TEST(MotionTest, MoveToJointAnglesFailsWhenNotConnected){
    
    std::vector<double> a{0,0,0,0,0,0,0};
    auto interface=KinovaInterface();
    bool validate =interface.moveToJointAngles(a);

    EXPECT_FALSE(validate);
}

// Test 2
TEST(MotionTest, MoveToJointAnglesFailsWithWrongVectorSize){

    std::vector<double>b{0,0,0,0,0,0};
    auto interface=KinovaInterface();
    interface.connect("1.1.1.1",10000);
    bool validate =interface.moveToJointAngles(b);
    EXPECT_FALSE(validate);

}

// Test 3
TEST(MotionTest, MoveToJointAnglesSucceedsWithValidAngles){

    std::vector<double> a{0,0,0,0,0,0,0};
    auto interface=KinovaInterface();
    interface.connect("1.1.1.1",10000);
    bool validate =interface.moveToJointAngles(a);
    EXPECT_TRUE(validate);
}

// Test 4
TEST(MotionTest, MoveBlockedWhenEStopActive){
    
    std::vector<double> a{0,0,0,0,0,0,0};
    auto interface=KinovaInterface();
    interface.connect("1.1.1.1",10000);
    interface.emergencyStop();
    bool validate =interface.moveToJointAngles(a);
    EXPECT_FALSE(validate);

}

// Test 5
TEST(StateTest, GetJointAnglesReturnsSevenValues){
    
    std::vector<double> a{0,0,0,0,0,0,0};
    auto interface=KinovaInterface();
    interface.connect("1.1.1.1",10000);
    interface.moveToJointAngles(a);
    std::vector<double> result=interface.getJointAngles();
    EXPECT_EQ(result.size(),7u);
}

// Test 6
TEST(StateTest, GetJointAnglesFailsWhenNotConnected){

    auto interface=KinovaInterface();
    std::vector<double> result=interface.getJointAngles();
    EXPECT_TRUE(result.empty());
}

// Test 7
TEST(SafetyTest, ClearEStopAllowsMotion){

    std::vector<double> a{0,0,0,0,0,0,0};
    
    auto interface=KinovaInterface();
    interface.connect("1.1.1.1",10000);
    interface.emergencyStop();
    interface.clearEmergencyStop();

    // No need to reconnect: see the notes
    bool validate =interface.moveToJointAngles(a);
    EXPECT_TRUE(validate);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}