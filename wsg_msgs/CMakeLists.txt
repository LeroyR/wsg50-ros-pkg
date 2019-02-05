cmake_minimum_required(VERSION 2.8.3)
project(wsg_msgs)

set(MSG_DEPS 
  std_msgs
  actionlib
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_message_files(DIRECTORY msg FILES
  Status.msg
  Command.msg
)

add_service_files(DIRECTORY srv FILES 
  SetValue.srv 
  GetGripperStatus.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Command.action
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
