cmake_minimum_required(VERSION 3.0.2)
project(palpate_tissue)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_eigen
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES palpate_tissue
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2_eigen
#  DEPENDS system_lib
)

include_directories(
# include
        ${catkin_INCLUDE_DIRS}
)

add_executable(palpate_1 src/palpate_1.cpp)
target_link_libraries(palpate_1
        ${catkin_LIBRARIES})
