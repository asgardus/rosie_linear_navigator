cmake_minimum_required(VERSION 2.8.3)
project(rosie_linear_navigator)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs tf)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs tf
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_linear_navigator src/linear_navigator_node.cpp)
target_link_libraries(rosie_linear_navigator ${catkin_LIBRARIES})
add_dependencies(rosie_linear_navigator geometry_msgs sensor_msgs nav_msgs tf)
