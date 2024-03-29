# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(esc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(hippo_msgs REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   hippocampus_common
#   hippocampus_msgs
#   path_planning
#   dynamic_reconfigure
# )

include_directories(
  include
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

add_executable(esc_commander_node src/esc_commander_node.cpp src/afro_esc.cpp)
ament_target_dependencies(esc_commander_node rclcpp hippo_msgs)

# Install Python executables
install(PROGRAMS
  nodes/esc_commander
  DESTINATION lib/${PROJECT_NAME}
)

install (
  TARGETS
  esc_commander_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
