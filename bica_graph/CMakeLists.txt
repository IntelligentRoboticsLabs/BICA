cmake_minimum_required(VERSION 2.8.3)
project(bica_graph)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  geometry_msgs
  bica_msgs
  tf
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
  src/${PROJECT_NAME}/node.cpp
  src/${PROJECT_NAME}/relation.cpp
  src/${PROJECT_NAME}/tf_relation.cpp
  src/${PROJECT_NAME}/conversions.cpp
  src/${PROJECT_NAME}/graph_publisher.cpp
  src/${PROJECT_NAME}/graph_listener.cpp
  src/${PROJECT_NAME}/graph_handler.cpp
)

set(BICA_GRAPH_HDRS
  include/${PROJECT_NAME}/graph.h
  include/${PROJECT_NAME}/node.h
  include/${PROJECT_NAME}/relation.h
  include/${PROJECT_NAME}/exceptions.h
  include/${PROJECT_NAME}/conversions.h
  include/${PROJECT_NAME}/graph_publisher.h
  include/${PROJECT_NAME}/graph_listener.h
  include/${PROJECT_NAME}/graph_handler.h
)

add_library(${PROJECT_NAME} ${BICA_GRAPH_SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

roslint_cpp(
  ${BICA_GRAPH_SRCS} ${BICA_GRAPH_HDRS}
  test/test_bica_graph.cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(${PROJECT_NAME}_test test/test_bica_graph.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} gtest)
  endif()

  add_rostest(test/test_bica_graph.test)
endif()
