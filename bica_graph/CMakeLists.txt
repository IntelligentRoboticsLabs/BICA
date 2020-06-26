cmake_minimum_required(VERSION 3.5)
project(bica_graph)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bica_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost 1.65.1 REQUIRED)

set(dependencies
    rclcpp
    bica_msgs
    geometry_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
)

include_directories(include)

set(BICAGRAPH_SOURCES
  src/bica_graph/Graph.cpp
  src/bica_graph/GraphNode.cpp
  src/bica_graph/TypedGraphNode.cpp
  src/bica_graph/Types.cpp
)

add_library(${PROJECT_NAME} SHARED ${BICAGRAPH_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(graph_terminal src/graph_terminal.cpp)
ament_target_dependencies(graph_terminal ${dependencies})
target_link_libraries(graph_terminal ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  graph_terminal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
