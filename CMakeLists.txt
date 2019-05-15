cmake_minimum_required(VERSION 2.8.3)
project(dji_m100_gazebo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp 
	rospy 
	std_msgs
	sensor_msgs
)

find_package(gazebo REQUIRED)
catkin_package(
	CATKIN_DEPENDS roscpp gazebo_ros
	LIBRARIES dji_ros_gazebo_control
	INCLUDE_DIRS
)

include_directories(
	${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_LIRARY_DIRS} ${catkin_LIBRARY_DIRS}
)

add_library(dji_ros_gazebo_control SHARED src/dji_ros_gazebo_control.cc)
target_link_libraries(dji_ros_gazebo_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

if(CMAKE_RELEASE_PREFIX STREQUAL "release")
	SET(CMAKE_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO})
	install(DIRECTORY launch
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
endif()

