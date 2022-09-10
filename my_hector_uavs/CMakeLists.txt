cmake_minimum_required(VERSION 2.8.3)
project(my_hector_uavs)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs
  tf
  hector_uav_msgs #Required for "enable_motors" service
)

catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hector_quadrotor_simple_movement src/hector_quadrotor_simple_movement.cpp)
target_link_libraries(hector_quadrotor_simple_movement ${catkin_LIBRARIES})

add_executable(hector_quadrotor_controller src/hector_quadrotor_controller.cpp)
target_link_libraries(hector_quadrotor_controller ${catkin_LIBRARIES})

add_executable(hector_quadrotor_controller_paths src/hector_quadrotor_controller_paths.cpp)
target_link_libraries(hector_quadrotor_controller_paths ${catkin_LIBRARIES})

#add_executable(two_hector_quadrotors_lf src/two_hector_quadrotors_lf.cpp)
#target_link_libraries(two_hector_quadrotors_lf ${catkin_LIBRARIES})

#add_executable(two_hector_quadrotors_lf_paths src/two_hector_quadrotors_lf_paths.cpp)
#target_link_libraries(two_hector_quadrotors_lf_paths ${catkin_LIBRARIES})


