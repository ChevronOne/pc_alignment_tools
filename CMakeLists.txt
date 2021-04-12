
cmake_minimum_required(VERSION 3.0.2)
project(pc_alignment_tools)

set(CMAKE_CXX_STANDARD 14)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
  message_filters
  tf2_ros
  tf2
  tf2_msgs
  nav_msgs
  eigen_conversions
  tf2_sensor_msgs
  geometry_msgs
  sensor_msgs

)
find_package(PCL REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp rospy roslib std_msgs message_filters tf2_ros tf2 tf2_msgs nav_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(point_to_point_aligner 
               src/pc_alignment_tools.hpp 
               src/point_to_point_aligner.cpp 
               src/get_inputs.cpp)
target_link_libraries(point_to_point_aligner 
                      ${PCL_LIBRARIES} ${catkin_LIBRARIES})



add_executable(nl_point_to_point_aligner
               src/pc_alignment_tools.hpp 
               src/nl_point_to_point_aligner.cpp 
               src/get_inputs.cpp)
target_link_libraries(nl_point_to_point_aligner 
                      ${PCL_LIBRARIES} ${catkin_LIBRARIES})



add_executable(point_to_plane_aligner 
               src/pc_alignment_tools.hpp 
               src/point_to_plane_aligner.cpp 
               src/get_inputs.cpp)
target_link_libraries(point_to_plane_aligner 
                      ${PCL_LIBRARIES} ${catkin_LIBRARIES})



add_executable(gicp_aligner 
               src/pc_alignment_tools.hpp 
               src/gicp_aligner.cpp 
               src/get_inputs.cpp)
target_link_libraries(gicp_aligner 
                      ${PCL_LIBRARIES} ${catkin_LIBRARIES})



add_executable(corres_filter_point_to_point_aligner 
               src/pc_alignment_tools.hpp 
               src/corres_filter_point_to_point_aligner.cpp 
               src/get_inputs.cpp)
target_link_libraries(corres_filter_point_to_point_aligner 
                      ${PCL_LIBRARIES} ${catkin_LIBRARIES})



add_executable(corres_filter_point_to_plane_aligner 
               src/pc_alignment_tools.hpp 
               src/corres_filter_point_to_plane_aligner.cpp 
               src/get_inputs.cpp)
target_link_libraries(corres_filter_point_to_plane_aligner
                      ${PCL_LIBRARIES} ${catkin_LIBRARIES})



