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

find_package(PkgConfig)

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
  message_filters
  std_msgs
  toaster_msgs
  dynamic_reconfigure
  tf
)

pkg_check_modules(pr2motion-genom3 REQUIRED pr2motion-genom3)


add_message_files(
   FILES
   MapStamped.msg
   Signal.msg
   Focus.msg
   AttentionStamped.msg
)

add_service_files(
  FILES
  Sync.srv
  Action.srv
)

generate_dynamic_reconfigure_options(
  cfg/stimulus_driven_dyn_param.cfg
  cfg/goal_directed_dyn_param.cfg
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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} $ENV{TOASTERLIB_DIR}/toaster-lib/include ${pr2motion-genom3_INCLUDE_DIRS} $ENV{ROBOTPKG_BASE}/include)

add_executable(stimulus_driven_attention src/stimulus_driven_attention.cpp)
target_link_libraries(stimulus_driven_attention ${catkin_LIBRARIES} $ENV{TOASTERLIB_DIR}/lib/libtoaster.so)
add_dependencies(stimulus_driven_attention ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(goal_directed_attention src/goal_directed_attention.cpp)
target_link_libraries(goal_directed_attention ${catkin_LIBRARIES} $ENV{TOASTERLIB_DIR}/lib/libtoaster.so)
add_dependencies(goal_directed_attention ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(sensitive_reorientation src/sensitive_reorientation.cpp)
target_link_libraries(sensitive_reorientation ${catkin_LIBRARIES})
add_dependencies(sensitive_reorientation ${catkin_EXPORTED_TARGETS})

add_executable(activity_states_faker src/activity_states_faker.cpp)
target_link_libraries(activity_states_faker ${catkin_LIBRARIES})
add_dependencies(activity_states_faker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(robot_observer_one src/robot_observer_one.cpp)
target_link_libraries(robot_observer_one ${catkin_LIBRARIES} $ENV{TOASTERLIB_DIR}/lib/libtoaster.so)
add_dependencies(robot_observer_one ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mocap_calib src/mocap_calib.cpp)
target_link_libraries(mocap_calib ${catkin_LIBRARIES})
add_dependencies(mocap_calib ${catkin_EXPORTED_TARGETS})
