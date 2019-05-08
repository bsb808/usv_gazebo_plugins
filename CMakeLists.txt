cmake_minimum_required(VERSION 2.8.3)
project(usv_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  hector_gazebo_plugins
  message_generation
  nav_msgs
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  )


## Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Boost
#find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)

## Eigen
find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#include_directories(${EIGEN_INCLUDE_DIRS}) 
#add_definitions(${EIGEN_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wall -Wextra")


########################
## Message generation ##
########################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  UsvDrive.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  )



###################################
## catkin specific configuration ##
###################################


if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
  catkin_package(  INCLUDE_DIRS include LIBRARIES gazebo_ros_elevator)
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  catkin_package(INCLUDE_DIRS include LIBRARIES gazebo_ros_harness)
endif()

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs nav_msgs
   DEPENDS 
   LIBRARIES
)

###########
## Build ##
###########


include_directories( include
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  )

add_library(usv_gazebo_dynamics_plugin
  src/usv_gazebo_dynamics_plugin.cpp
  )

target_link_libraries(usv_gazebo_dynamics_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(usv_gazebo_thrust_plugin
  src/usv_gazebo_thrust_plugin.cpp
  )

add_dependencies(usv_gazebo_thrust_plugin
  usv_gazebo_plugins_generate_messages_cpp
)

target_link_libraries(usv_gazebo_thrust_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_dependencies(usv_gazebo_thrust_plugin
  usv_gazebo_plugins_generate_messages_cpp
)

add_library(usv_gazebo_wind_plugin
  src/usv_gazebo_wind_plugin.cpp
  )

target_link_libraries(usv_gazebo_wind_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
)


#add_executable(ugdp_test
#  test/ugdp_test.cpp
#  )
#target_link_libraries(ugdp_test
#  usv_gazebo_dynamics_plugin
#  )

add_executable(frame_test
  test/frame_test.cpp
  )

target_link_libraries(frame_test
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
  
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS usv_gazebo_plugins usv_gazebo_plugins_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_usv_gazebo_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
