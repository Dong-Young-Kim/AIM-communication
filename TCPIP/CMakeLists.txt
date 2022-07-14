cmake_minimum_required(VERSION 3.0.2)
project(TCPIP)


find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)

add_message_files(
  FILES
  object_msg.msg
  object_msg_arr.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(TCPIP src/C_TCPIP.cpp)
target_link_libraries(TCPIP ${catkin_LIBRARIES})

add_executable(TCPIP_test src/C_client_test.cpp)
target_link_libraries(TCPIP_test ${catkin_LIBRARIES})

