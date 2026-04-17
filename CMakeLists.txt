cmake_minimum_required(VERSION 3.16)
project(kinova_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

include_directories(include mock)

# Shared wrapper source
set(WRAPPER_SRC src/KinovaInterface.cpp)

# Kinematics library path
set(KINEMATICS_DIR ${CMAKE_SOURCE_DIR}/../kinova_kinematics)

# Test targets
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test_connection tests/test_connection.cpp ${WRAPPER_SRC})
target_compile_definitions(test_connection PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_connection GTest::GTest GTest::Main pthread)

add_executable(test_motion tests/test_motion.cpp ${WRAPPER_SRC})
target_compile_definitions(test_motion PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_motion GTest::GTest GTest::Main pthread)

add_executable(test_gripper tests/gripper_test.cpp ${WRAPPER_SRC})
target_compile_definitions(test_gripper PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_gripper GTest::GTest GTest::Main pthread)

add_executable(test_trajectory tests/TrajectoryTest.cpp ${WRAPPER_SRC})
target_compile_definitions(test_trajectory PRIVATE USE_KORTEX_MOCK)
target_link_libraries(test_trajectory GTest::GTest GTest::Main pthread)

# =============================================================================
# Real Hardware Test Targets (no USE_KORTEX_MOCK)
# =============================================================================
set(KORTEX_DIR /home/sahil/kinova_learning/kortex/api_cpp/examples/kortex_api)

set(KORTEX_INCLUDES
    ${KORTEX_DIR}/include
    ${KORTEX_DIR}/include/client
    ${KORTEX_DIR}/include/common
    ${KORTEX_DIR}/include/messages
    ${KORTEX_DIR}/include/client_stubs
)

set(KORTEX_LIB ${KORTEX_DIR}/lib/release/libKortexApiCpp.a)

add_executable(hw_test_connection tests/hardware/test_hw_connection.cpp ${WRAPPER_SRC})
target_include_directories(hw_test_connection SYSTEM PRIVATE ${KORTEX_INCLUDES})
target_compile_definitions(hw_test_connection PRIVATE _OS_UNIX)
target_compile_options(hw_test_connection PRIVATE -Wno-error)
target_link_libraries(hw_test_connection ${KORTEX_LIB} pthread)

add_executable(hw_test_motion tests/hardware/test_hw_motion.cpp ${WRAPPER_SRC})
target_include_directories(hw_test_motion SYSTEM PRIVATE ${KORTEX_INCLUDES})
target_compile_definitions(hw_test_motion PRIVATE _OS_UNIX)
target_compile_options(hw_test_motion PRIVATE -Wno-error)
target_link_libraries(hw_test_motion ${KORTEX_LIB} pthread)

add_executable(hw_test_state tests/hardware/test_hw_state.cpp ${WRAPPER_SRC})
target_include_directories(hw_test_state SYSTEM PRIVATE ${KORTEX_INCLUDES})
target_compile_definitions(hw_test_state PRIVATE _OS_UNIX)
target_compile_options(hw_test_state PRIVATE -Wno-error)
target_link_libraries(hw_test_state ${KORTEX_LIB} pthread)

add_executable(hw_test_gripper tests/hardware/test_hw_gripper.cpp ${WRAPPER_SRC})
target_include_directories(hw_test_gripper SYSTEM PRIVATE ${KORTEX_INCLUDES})
target_compile_definitions(hw_test_gripper PRIVATE _OS_UNIX)
target_compile_options(hw_test_gripper PRIVATE -Wno-error)
target_link_libraries(hw_test_gripper ${KORTEX_LIB} pthread)

add_executable(hw_test_estop tests/hardware/test_hw_estop.cpp ${WRAPPER_SRC})
target_include_directories(hw_test_estop SYSTEM PRIVATE ${KORTEX_INCLUDES})
target_compile_definitions(hw_test_estop PRIVATE _OS_UNIX)
target_compile_options(hw_test_estop PRIVATE -Wno-error)
target_link_libraries(hw_test_estop ${KORTEX_LIB} pthread)

add_executable(hw_test_fk_ik tests/hardware/test_hw_fk_ik.cpp 
    ${WRAPPER_SRC}
    ${KINEMATICS_DIR}/src/KinovaKinematics.cpp)
target_include_directories(hw_test_fk_ik SYSTEM PRIVATE 
    ${KORTEX_INCLUDES}
    ${KINEMATICS_DIR}/include)
target_compile_definitions(hw_test_fk_ik PRIVATE _OS_UNIX)
target_compile_options(hw_test_fk_ik PRIVATE -Wno-error)
target_link_libraries(hw_test_fk_ik ${KORTEX_LIB} pthread Eigen3::Eigen)