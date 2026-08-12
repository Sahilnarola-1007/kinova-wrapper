cmake_minimum_required(VERSION 3.16)
project(kinova_wrapper)

# =============================================================================
# C++ standard & warnings
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Build options
# =============================================================================
# USE_KORTEX_MOCK: ON  = build with mock SDK (desk development, default)
#                  OFF = link real Kortex SDK (lab/hardware)
option(USE_KORTEX_MOCK "Build against mock Kortex SDK" ON)

# Kortex SDK location (override with -DKORTEX_DIR=/custom/path if needed)
set(KORTEX_DIR "/home/sahil/kinova_learning/kortex/api_cpp/examples/kortex_api"
    CACHE PATH "Path to Kortex API SDK root")

# =============================================================================
# Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)

# =============================================================================
# Main library: libkinova_wrapper.so
# =============================================================================
add_library(kinova_wrapper SHARED src/KinovaInterface.cpp)

target_include_directories(kinova_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(USE_KORTEX_MOCK)
  message(STATUS "kinova_wrapper: building with MOCK Kortex SDK")
  target_compile_definitions(kinova_wrapper PUBLIC USE_KORTEX_MOCK)
else()
  message(STATUS "kinova_wrapper: building with REAL Kortex SDK at ${KORTEX_DIR}")
  if(NOT EXISTS "${KORTEX_DIR}/lib/release/libKortexApiCpp.a")
    message(FATAL_ERROR "Kortex SDK not found at ${KORTEX_DIR}. "
                        "Install it or pass -DKORTEX_DIR=/correct/path")
  endif()
  target_include_directories(kinova_wrapper SYSTEM PUBLIC
    ${KORTEX_DIR}/include
    ${KORTEX_DIR}/include/client
    ${KORTEX_DIR}/include/common
    ${KORTEX_DIR}/include/messages
    ${KORTEX_DIR}/include/client_stubs
  )
  target_compile_definitions(kinova_wrapper PUBLIC _OS_UNIX)
  target_link_libraries(kinova_wrapper PUBLIC
    ${KORTEX_DIR}/lib/release/libKortexApiCpp.a
  )
endif()

target_link_libraries(kinova_wrapper PUBLIC pthread)

# =============================================================================
# Install: library + headers + CMake export targets
# =============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS kinova_wrapper
  EXPORT kinova_wrapperTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(kinova_wrapperTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# =============================================================================
# Tests (gtest — only built when BUILD_TESTING is ON)
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Mock-mode unit tests (only meaningful when library built with mock)
  if(USE_KORTEX_MOCK)
    ament_add_gtest(test_connection tests/test_connection.cpp)
    target_link_libraries(test_connection kinova_wrapper)

    ament_add_gtest(test_motion tests/test_motion.cpp)
    target_link_libraries(test_motion kinova_wrapper)

    ament_add_gtest(test_gripper tests/gripper_test.cpp)
    target_link_libraries(test_gripper kinova_wrapper)

    ament_add_gtest(test_trajectory tests/TrajectoryTest.cpp)
    target_link_libraries(test_trajectory kinova_wrapper)

    ament_add_gtest(test_velocity tests/test_velocity.cpp)
    target_link_libraries(test_velocity kinova_wrapper)
  endif()
endif()

# Hardware tests (real SDK only, manual run — never in CI)
if(NOT USE_KORTEX_MOCK)
  find_package(kinova_kinematics REQUIRED)
  find_package(Eigen3 REQUIRED)

  add_executable(test_hw_fk_ik tests/hardware/test_hw_fk_ik.cpp)
  target_link_libraries(test_hw_fk_ik
    kinova_wrapper
    kinova_kinematics::kinova_kinematics
    Eigen3::Eigen
  )
endif()

ament_package()