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

find_package(Boost REQUIRED COMPONENTS)

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
  toaster_msgs
  dynamic_reconfigure
)

add_message_files(
   FILES
   StampedMap.msg
)

add_service_files(
  FILES
  InhibitionOfReturn.srv
  Sync.srv
)

generate_dynamic_reconfigure_options(
  cfg/reactive_dyn_param.cfg
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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} $ENV{TOASTERLIB_DIR}/include)

add_executable(salient_stimuli_selection src/salient_stimuli_selection.cpp)
target_link_libraries(salient_stimuli_selection ${catkin_LIBRARIES} $ENV{TOASTERLIB_DIR}/lib/libtoaster.so)
add_dependencies(salient_stimuli_selection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(cognitive_orientation src/cognitive_orientation.cpp)
target_link_libraries(cognitive_orientation ${catkin_LIBRARIES} $ENV{TOASTERLIB_DIR}/lib/libtoaster.so)
add_dependencies(cognitive_orientation ${catkin_EXPORTED_TARGETS})

add_executable(sensitive_reorientation src/sensitive_reorientation.cpp)
target_link_libraries(sensitive_reorientation ${catkin_LIBRARIES})
add_dependencies(sensitive_reorientation ${catkin_EXPORTED_TARGETS})