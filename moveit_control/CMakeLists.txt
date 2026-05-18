cmake_minimum_required(VERSION 3.8)
project(moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  controller_manager_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(controller_manager_msgs REQUIRED)

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


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  # Headers from the binary-installed moveit_servo
  ${moveit_servo_INCLUDE_DIRS}
)

# Build local modified pose_tracking as a library
add_library(local_pose_tracking SHARED
  src/pose_tracking.cpp
)
ament_target_dependencies(local_pose_tracking
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  moveit_servo
)
target_include_directories(local_pose_tracking PUBLIC
  include
  /opt/ros/$ENV{ROS_DISTRO}/include/moveit_servo
)

# ---- Executable ----
add_executable(realtime_pose_tracking_node
  src/realtime_pose_tracking_node.cpp
)

# For binary installs, link against the .so files directly by name
target_link_libraries(realtime_pose_tracking_node
  local_pose_tracking
  ${moveit_servo_LIBRARIES}   
)

ament_target_dependencies(realtime_pose_tracking_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  moveit_servo
  controller_manager_msgs
)
# ---- Install ----
install(
  TARGETS
    local_pose_tracking
    realtime_pose_tracking_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  moveit_control/moveit_target_pose_control.py
  moveit_control/online_target_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
