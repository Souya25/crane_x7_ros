cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_robot_design3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CN_DEPENDS message_runtime
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
  DEPENDS
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLDUE_DIRS})

#############
## Install ##
#############

add_executable(cam src/opencv.cpp)
target_link_libraries(cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

