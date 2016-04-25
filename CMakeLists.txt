cmake_minimum_required(VERSION 2.8.3)
project(head_manager)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++1y")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  pr2_controllers_msgs
  roscpp
  actionlib_msgs
  std_msgs
  rospy
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
   FILES
   StampedMap.msg
)

add_service_files(
  FILES
  InhibitionOfReturn.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(salient_stimuli_selection src/salient_stimuli_selection.cpp)
target_link_libraries(salient_stimuli_selection ${catkin_LIBRARIES})
add_dependencies(salient_stimuli_selection ${catkin_EXPORTED_TARGETS})

add_executable(head_driver src/head_driver.cpp)
target_link_libraries(head_driver ${catkin_LIBRARIES})
add_dependencies(head_driver ${catkin_EXPORTED_TARGETS})