################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(cxx_pkg)



if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(study_msgs REQUIRED)


################################################################################
# Build
################################################################################
# include_directories(
#   include
# )

# add_library(${PROJECT_NAME}_lib
#   "src/devices/motor_power.cpp"
#   "src/devices/sound.cpp"
#   "src/devices/reset.cpp"

#   "src/diff_drive_controller.cpp"
#   "src/dynamixel_sdk_wrapper.cpp"
#   "src/odometry.cpp"
#   "src/turtlebot3.cpp"

#   "src/sensors/battery_state.cpp"
#   "src/sensors/imu.cpp"
#   "src/sensors/joint_state.cpp"
#   "src/sensors/sensor_state.cpp"
# )

# target_link_libraries(${PROJECT_NAME}_lib)
# ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

add_executable(parking_node src/parking.cpp)
ament_target_dependencies(parking_node rclcpp sensor_msgs geometry_msgs)

add_executable(cmd_vel_pub_node src/cmd_vel_pub.cpp)
ament_target_dependencies(cmd_vel_pub_node rclcpp geometry_msgs)

add_executable(laser_sub_node src/laser_sub.cpp)
ament_target_dependencies(laser_sub_node rclcpp sensor_msgs)

add_executable(move_shape_node src/move_shape.cpp)
ament_target_dependencies(move_shape_node rclcpp study_msgs geometry_msgs)

################################################################################
# Install
################################################################################

# param or launch
# install(DIRECTORY param
#   DESTINATION share/${PROJECT_NAME}
# )

# node
install(TARGETS parking_node cmd_vel_pub_node laser_sub_node move_shape_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
# ament_export_include_directories(include)
ament_package()