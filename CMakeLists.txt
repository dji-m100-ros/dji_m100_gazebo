cmake_minimum_required(VERSION 2.8.3)
project(dji_m100_gazebo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
)

find_package(gazebo REQUIRED)
catkin_package(
)

include_directories(
	${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_LIRARY_DIRS} ${catkin_LIBRARY_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)