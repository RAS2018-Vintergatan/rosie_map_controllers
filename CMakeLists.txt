cmake_minimum_required(VERSION 2.8.3)
project(rosie_map_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs phidgets geometry_msgs visualization_msgs sensor_msgs tf message_filters laser_geometry pcl_ros)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp rospy std_msgs phidgets geometry_msgs visualization_msgs sensor_msgs tf message_filters laser_geometry pcl_ros
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_laser_publisher src/rosie_laser_publisher.cpp)
add_executable(rosie_map_provider src/rosie_map_provider.cpp)
add_executable(rosie_map_localizer src/rosie_map_localizer.cpp)
#add_executable(rosie_obj2map src/rosie_obj2map.cpp)
#add_executable(rosie_map_updater src/rosie_map_updater.cpp)
#add_executable(rosie_rrt src/rosie_rrt.cpp)
#target_link_libraries(rosie_rrt ${catkin_LIBRARIES})
#target_link_libraries(rosie_map_updater ${catkin_LIBRARIES})
target_link_libraries(rosie_laser_publisher ${catkin_LIBRARIES})
target_link_libraries(rosie_map_localizer ${catkin_LIBRARIES})
#target_link_libraries(rosie_obj2map ${catkin_LIBRARIES})
target_link_libraries(rosie_map_provider ${catkin_LIBRARIES})
#add_dependencies(rosie_laser_publisher rosie_obj2map rosie_map_provider std_msgs phidgets geometry_msgs visualization_msgs sensor_msgs tf message_filters laser_geometry )
add_dependencies(rosie_laser_publisher rosie_map_provider rosie_map_localizer std_msgs phidgets geometry_msgs visualization_msgs sensor_msgs pcl_ros tf message_filters laser_geometry )
