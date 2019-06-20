cmake_minimum_required(VERSION 2.8.3)
project(bica_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)


add_message_files(
   FILES
   Node.msg
   Edge.msg
   Graph.msg
   GraphUpdate.msg
 )

 add_service_files(
    FILES
    UpdateGraph.srv
)
  
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

install(DIRECTORY include/bica_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
