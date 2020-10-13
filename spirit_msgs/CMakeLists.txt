cmake_minimum_required(VERSION 3.0.2)
project(spirit_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
 	FILES
	StateEstimate.msg
	TorqueCommand.msg
	BodyPlan.msg
	FootstepPlan.msg
	Footstep.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	sensor_msgs
	std_msgs
	nav_msgs
)