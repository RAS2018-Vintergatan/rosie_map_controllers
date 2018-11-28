cmake_minimum_required(VERSION 2.8.3)
project(rosie_map_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  phidgets
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf
  laser_geometry
  pcl_ros
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets nav_msgs geometry_msgs visualization_msgs sensor_msgs tf message_filters laser_geometry pcl_ros
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_laser_publisher src/rosie_laser_publisher.cpp)
add_executable(rosie_map_localizer src/rosie_map_localizer.cpp)
add_executable(rosie_map_provider src/rosie_map_provider.cpp)
target_link_libraries(rosie_laser_publisher ${catkin_LIBRARIES})
target_link_libraries(rosie_map_localizer ${catkin_LIBRARIES})
target_link_libraries(rosie_map_provider ${catkin_LIBRARIES})
add_dependencies(rosie_laser_publisher ${catkin_EXPORTED_TARGETS})
add_dependencies(rosie_map_provider ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rosie_map_localizer ${catkin_EXPORTED_TARGETS})
