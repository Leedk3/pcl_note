cmake_minimum_required(VERSION 2.8.3)
project(frenet_c)

#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_C_STANDARD 11)

#add_definitions("-std=c++0x -Wall -Werror")
add_definitions("-std=c++1z")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  tf
  cv_bridge
#  pcl_ros
#  pcl_conversions
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/frenet_c_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

#Hear start GTSAM , PCL, OpenCV dependancy

#find_package(GTSAM REQUIRED QUIET)
#find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS cloud_msgs
#  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
#	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
#	${PCL_LIBRARY_DIRS}
#	${GTSAM_LIBRARY_DIRS}
)

