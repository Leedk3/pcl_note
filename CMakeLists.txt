cmake_minimum_required(VERSION 2.8.3)
project(pcl_example)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  tf
  pcl_ros
  pcl_conversions
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(${PROJECT_NAME}_node src/transform_node.cpp)
add_executable(transform_node src/transform_node.cpp)

add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(transform_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

#Hear start GTSAM , PCL, OpenCV dependancy

#find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
#find_package(OpenCV REQUIRED QUIET)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
#	${OpenCV_INCLUDE_DIRS}
#	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
#	${GTSAM_LIBRARY_DIRS}
)

