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
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs std_srvs
#  DEPENDS 
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(testnode src/main.cpp src/can/can_device.cpp src/can/wsg_can.cpp src/gripper_data.cpp)
target_link_libraries(testnode ${catkin_LIBRARIES})

##  qt 5.10 has can support 
#find_package(Qt5Core REQUIRED)
#qt5_use_modules(testnode Core SerialPort) qt 5.10


#add_compile_flags(wsg_driver_can -g -Wall)
#target_link_libraries(wsg_driver_can pcan)
