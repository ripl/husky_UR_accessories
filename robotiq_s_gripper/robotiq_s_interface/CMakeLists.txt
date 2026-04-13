cmake_minimum_required(VERSION 3.0.2)
project(robotiq_s_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  robotiq_s_model_articulated_msgs
  robotiq_s_model_control
)

catkin_python_setup()

catkin_package()
