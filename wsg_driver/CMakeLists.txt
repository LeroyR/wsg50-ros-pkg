cmake_minimum_required(VERSION 2.8.3)
project(wsg_driver)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
	add_compile_options(-std=c++14)
else ()
	set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  wsg_msgs
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs std_srvs wsg_msgs
#  DEPENDS 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# wsg_driver_TCP version
set(DRIVER_SOURCES
  src/checksum.cpp include/wsg_driver/checksum.h
  src/cmd.c include/wsg_driver/cmd.h
  src/common.cpp include/wsg_driver/common.h
  src/functions.cpp include/wsg_driver/functions.h
  src/interface.cpp include/wsg_driver/interface.h
  src/gripper_communication.cpp include/wsg_driver/gripper_communication.h
  src/gripper_action_server.cpp include/wsg_driver/gripper_action_server.h
  src/gripper_standard_action_server.cpp include/wsg_driver/gripper_standard_action_server.h
  src/gripper_socket.cpp include/wsg_driver/gripper_socket.h
  include/wsg_driver/node_state.h
  src/main.cpp
  src/msg.c include/wsg_driver/msg.h
  src/serial.c include/wsg_driver/serial.h
  src/tcp.c include/wsg_driver/tcp.h
  src/udp.c include/wsg_driver/udp.h)

# wsg_driver_CAN version
set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(wsg_driver_ip src/main.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg_driver_ip ${catkin_LIBRARIES})
add_dependencies(wsg_driver_ip wsg_msgs_generate_messages_cpp)

#add_executable(wsg_driver_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
#add_executable(wsg_driver_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)
#add_compile_flags(wsg_driver_can -g -Wall)
#target_link_libraries(wsg_driver_can pcan)
