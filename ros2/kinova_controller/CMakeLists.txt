cmake_minimum_required(VERSION 3.8)
project(kinova_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kinova_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# NOTE: Paths are absolute until the wrapper is packaged as a standalone ROS2 package.
# For portability, set these via environment variables or CMake cache.
set(KINOVA_WRAPPER_DIR /home/sahil/kinova_learning/wrapper)
set(KORTEX_API_DIR /home/sahil/kinova_learning/kortex_api)

add_library(kinova_lifecycle_component SHARED
  src/KinovaLifecycleController.cpp
  ${KINOVA_WRAPPER_DIR}/src/KinovaInterface.cpp
  )

# Including required direcores of libraries
target_include_directories(kinova_lifecycle_component PRIVATE
  include
  ${KINOVA_WRAPPER_DIR}/include
  ${KORTEX_API_DIR}/include
  ${KORTEX_API_DIR}/include/client
  ${KORTEX_API_DIR}/include/client_stubs
  ${KORTEX_API_DIR}/include/messages
  ${KORTEX_API_DIR}/include/common
)

option(USE_MOCK_SDK "Use mock Kortex SDK for testing" OFF)
if(USE_MOCK_SDK)
  target_compile_definitions(kinova_lifecycle_component PRIVATE USE_KORTEX_MOCK)
endif()

target_link_directories(kinova_lifecycle_component PRIVATE
  ${KORTEX_API_DIR}/lib/release
)

target_link_libraries(kinova_lifecycle_component
  KortexApiCpp
  pthread
)

ament_target_dependencies(kinova_lifecycle_component
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  rclcpp_components
  rclcpp_action
  kinova_interfaces
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS kinova_lifecycle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(kinova_lifecycle_component
  PLUGIN "kinova_ros2::KinovaLifecycleController"
  EXECUTABLE kinova_controller_standalone
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})


ament_package()
