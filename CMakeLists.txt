cmake_minimum_required(VERSION 2.8.3)

project(hands_on_intervention)



find_package(catkin REQUIRED COMPONENTS
 
  rospy
  std_msgs
  message_generation
)


add_service_files(
FILES
intervention.srv
)

generate_messages(
DEPENDENCIES
std_msgs
)
