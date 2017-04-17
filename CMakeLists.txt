cmake_minimum_required(VERSION 2.8.3)

project(simple)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs move_base_msgs actionlib openslam_gmapping)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(eigens eigens.cpp)

target_link_libraries(eigens ${catkin_LIBRARIES})
