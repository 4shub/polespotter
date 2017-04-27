cmake_minimum_required(VERSION 2.8.3)

project(polespotter)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs move_base_msgs actionlib openslam_gmapping)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(main src/main.cpp)
add_executable(movement src/movement.cpp)
add_executable(scanner src/scanner.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(movement ${catkin_LIBRARIES})
target_link_libraries(scanner ${catkin_LIBRARIES})


#add_executable(eigens eigens.cpp)

#target_link_libraries(eigens ${catkin_LIBRARIES})
