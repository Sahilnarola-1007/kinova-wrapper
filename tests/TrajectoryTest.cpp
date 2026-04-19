#include<gtest/gtest.h>
#include"kinova_wrapper/KinovaInterface.hpp"

using namespace kinova_wrapper;
  
// Test 1
TEST(TrajectoryTest, ValidTrajectory) {
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint> waypoints = {
        {{0.0, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0}, 1.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };

    EXPECT_TRUE(kinova.executeTrajectory(waypoints));
}

// Test 2
TEST(TrajectoryTest, EmptyTrajectory) {
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint> waypoints = {};

    EXPECT_FALSE(kinova.executeTrajectory(waypoints));
}


//Test 3
TEST(TrajectoryTest,WrongJointCount){
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint>waypoints{
        {{0.0, 0.1, 0.0, -0.5, 0.0, -0.3}, 1.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };
    EXPECT_FALSE(kinova.executeTrajectory(waypoints));
}

//Test 4
TEST(TrajectoryTest,NonMonotonicTime){
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint>waypoints{
        {{0.0, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 1.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };
    EXPECT_FALSE(kinova.executeTrajectory(waypoints));
}

//Test 5
TEST(TrajectoryTest, JointLimitViolation) {
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint> waypoints = {
        {{0.0, 2.5, 0.0, -0.5, 0.0, -0.3, 10.0}, 1.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };

    EXPECT_FALSE(kinova.executeTrajectory(waypoints));
}

//Test 6
TEST(TrajectoryTest, FeedbackCallbackFires) {
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint> waypoints = {
        {{0.0, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0}, 1.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };

    int count = 0;  // counter lives in the test

    // Lambda captures 'count' by reference — each call increments it
    auto callback = [&count](const std::vector<double>& joints, double progress) {
        count++;  // every time executeTrajectory fires the callback, this runs
    };

    EXPECT_TRUE(kinova.executeTrajectory(waypoints, callback));
    EXPECT_EQ(count, 3);  // 3 waypoints → callback fired 3 times
}

//Test 7
TEST(TrajectoryTest, AsyncReturnsTrue) {
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    std::vector<kinova_wrapper::TrajectoryPoint> waypoints = {
        {{0.0, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0}, 1.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };
    
    auto future=kinova.executeTrajectoryAsync(waypoints);

    EXPECT_TRUE(future.get());

}

//Test 8
TEST(TrajectoryTest, EStopRejectsTrajectory) {
    kinova_wrapper::KinovaInterface kinova;
    ASSERT_TRUE(kinova.connect("192.168.1.10"));

    kinova.emergencyStop();

    std::vector<kinova_wrapper::TrajectoryPoint> waypoints = {
        {{0.0, 0.1, 0.0, -0.5, 0.0, -0.3, 0.0}, 1.0},
        {{0.0, 0.2, 0.0, -0.5, 0.0, -0.3, 0.0}, 2.0},
        {{0.0, 0.3, 0.0, -0.5, 0.0, -0.3, 0.0}, 3.0},
    };

    EXPECT_FALSE(kinova.executeTrajectory(waypoints));

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
