cmake_minimum_required(VERSION 3.5)
project(bica_graph)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bica_msgs REQUIRED)

set(dependencies
    rclcpp
    bica_msgs
)

include_directories(include)

set(BICAGRAPH_SOURCES
  src/bica_graph/Graph.cpp
  src/bica_graph/GraphNode.cpp
  src/bica_graph/Types.cpp
)

add_library(${PROJECT_NAME} SHARED ${BICAGRAPH_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
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
