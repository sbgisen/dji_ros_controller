cmake_minimum_required(VERSION 3.8)
project(dji_ros_controller)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# Find libraries
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${controller_interface_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${can_msgs_INCLUDE_DIRS}
  ${controller_manager_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${controller_manager_msgs_INCLUDE_DIRS}
  ${ros2_socketcan_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()

file(
  GLOB MSG_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  msg/*.msg
)

# Specify additional locations of header files
include_directories(include)

# Declare a C++ executable
file(
  GLOB
  DJI_SOURCES
  "src/dji_can_communication.cpp"
  "src/M2006Ros2.cpp"
)

ament_auto_add_executable(
  M2006_ros2
  src/M2006Ros2.cpp
  src/dji_can_communication.cpp
)

ament_target_dependencies(
  M2006_ros2
  rclcpp
  sensor_msgs
  can_msgs
  controller_manager
  hardware_interface
  control_toolbox
  joint_state_broadcaster
)

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/M2006Ros2.cpp
  src/dji_can_communication.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface M2006_hardware.xml)

ament_auto_package()
