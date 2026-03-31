cmake_minimum_required(VERSION 3.16)
project(kinova_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

include_directories(include mock)

# Shared wrapper source
set(WRAPPER_SRC src/KinovaInterface.cpp)

# Test targets
find_package(GTest REQUIRED)

add_executable(test_connection tests/test_connection.cpp ${WRAPPER_SRC})
target_compile_definitions(test_connection PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_connection GTest::GTest GTest::Main pthread)

add_executable(test_motion tests/test_motion.cpp ${WRAPPER_SRC})
target_compile_definitions(test_motion PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_motion GTest::GTest GTest::Main pthread)

add_executable(test_gripper tests/gripper_test.cpp ${WRAPPER_SRC})
target_compile_definitions(test_gripper PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_gripper GTest::GTest GTest::Main pthread)
