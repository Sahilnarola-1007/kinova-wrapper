#include<gtest/gtest.h>
#include"kinova_interface/KinovaInterface.hpp"

using namespace kinova_wrapper;

//Test 1
TEST(ConnectionTest,ConnectFailsWithInvalidIP){
    auto interface=KinovaInterface();
    bool connection=interface.connect("",10000);

    // Google has simple assertion macros which automatically assert
    EXPECT_FALSE(connection);
    EXPECT_FALSE(interface.isConnected());
}

//Test 2
TEST(ConnectionTest,ConnectSucceedsWithvalidIP){
    auto interface=KinovaInterface();
    bool connection=interface.connect("1.1.1.1",10000);
    EXPECT_TRUE(connection);
    EXPECT_TRUE(interface.isConnected());

}

//Test 3
TEST(ConnectionTest,IsConnectedFalseAfterDisconnect){
    auto interface=KinovaInterface();
    interface.connect("1.1.1.1",10000);
    interface.disconnect();
    EXPECT_FALSE(interface.isConnected());

}

//Test 4
TEST(ConnectionTest, DisconnectWhenNotConnectedIsSafe){

    auto interface=KinovaInterface();
    interface.disconnect();
    EXPECT_FALSE(interface.isConnected());
    

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}