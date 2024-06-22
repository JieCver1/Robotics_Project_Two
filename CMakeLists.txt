cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs sensor_msgs tf dynamic_reconfigure move_base_msgs actionlib actionlib_msgs) 
add_action_files(DIRECTORY src FILES Move_base.action)
generate_messages(DEPENDENCIES actionlib_msgs move_base_msgs)
## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ugv_odom_to_tf src/ugv_odom_to_tf.cpp)
target_link_libraries(ugv_odom_to_tf ${catkin_LIBRARIES})
add_executable(goal_publisher src/goal_publisher.cpp)
target_link_libraries(goal_publisher ${catkin_LIBRARIES})
