cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_gazebo_plugins)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS std_msgs gazebo_plugins actionlib tf image_transport control_msgs trajectory_msgs geometry_msgs sensor_msgs roscpp gazebo_ros robotiq_2f_gripper_control dynamic_reconfigure)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

catkin_package(
   DEPENDS gazebo_plugins gazebo_ros robotiq_2f_gripper_control roscpp dynamic_reconfigure
)

add_library(Robotiq2fPlugin src/Robotiq2fPlugin.cpp)
set_target_properties(Robotiq2fPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(Robotiq2fPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(Robotiq2fPlugin ${catkin_LIBRARIES})
add_dependencies(Robotiq2fPlugin robotiq_2f_gripper_control_gencpp ${PROJECT_NAME}_gencfg)

install(TARGETS Robotiq2fPlugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)

