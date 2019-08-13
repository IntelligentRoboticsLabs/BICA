cmake_minimum_required(VERSION 2.8.3)
project(bica_graph)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  geometry_msgs
  bica_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  bica_planning
)

find_package(rostest REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(BICA_GRAPH_SRCS
  src/${PROJECT_NAME}/graph.cpp
  src/${PROJECT_NAME}/edge.cpp
  src/${PROJECT_NAME}/node.cpp
  src/${PROJECT_NAME}/conversions.cpp
  src/${PROJECT_NAME}/graph_client.cpp
  src/${PROJECT_NAME}/graph_server.cpp
  src/${PROJECT_NAME}/graph.cpp
)

set(BICA_GRAPH_HDRS
  include/${PROJECT_NAME}/graph.h
  include/${PROJECT_NAME}/node.h
  include/${PROJECT_NAME}/edge.h
  include/${PROJECT_NAME}/conversions.h
  include/${PROJECT_NAME}/exceptions.h
  include/${PROJECT_NAME}/macros.h
  include/${PROJECT_NAME}/graph_client.h
)

add_library(${PROJECT_NAME} ${BICA_GRAPH_SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(graph_master_node src/graph_master_node.cpp)
add_dependencies(graph_master_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_master_node ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(graph_kms_extractor_node src/graph_kms_extractor_node.cpp)
add_dependencies(graph_kms_extractor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_kms_extractor_node ${catkin_LIBRARIES} ${PROJECT_NAME} )


add_executable(test_publisher_node test/test_publisher_node.cpp)
add_dependencies(test_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_publisher_node ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(test_subscriber_node test/test_subscriber_node.cpp)
add_dependencies(test_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_subscriber_node ${catkin_LIBRARIES} ${PROJECT_NAME} )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/explicit,-runtime/references,-build/c++11")

roslint_cpp(
  ${BICA_GRAPH_SRCS} ${BICA_GRAPH_HDRS}
  test/test_bica_graph.cpp
  test/test_bica_graph_clientserver.cpp
  src/graph_master_node.cpp
  src/graph_kms_extractor_node.cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(${PROJECT_NAME}_test test/test_bica_graph.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)
  endif()
  add_executable(${PROJECT_NAME}_test_clientserver test/test_bica_graph_clientserver.cpp)
  if(TARGET ${PROJECT_NAME}_test_clientserver)
    target_link_libraries(${PROJECT_NAME}_test_clientserver ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)
  endif()

  add_rostest(test/test_bica_graph.test)
endif()
