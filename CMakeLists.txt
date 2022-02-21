cmake_minimum_required(VERSION 3.0.2)
project(esc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hippocampus_common
  hippocampus_msgs
  path_planning
  dynamic_reconfigure
)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# )

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/esc_commander
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
