cmake_minimum_required(VERSION 3.5)
project(bica)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bica_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    bica_msgs
    ament_index_cpp
)

include_directories(include)

set(BICA_SOURCES
  src/bica/Component.cpp
)

add_library(${PROJECT_NAME} SHARED ${BICA_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(component_A_node src/component_A_node.cpp)
ament_target_dependencies(component_A_node ${dependencies})
target_link_libraries(component_A_node ${PROJECT_NAME})

add_executable(component_B_node src/component_B_node.cpp)
ament_target_dependencies(component_B_node ${dependencies})
target_link_libraries(component_B_node ${PROJECT_NAME})

add_executable(component_C_node src/component_C_node.cpp)
ament_target_dependencies(component_C_node ${dependencies})
target_link_libraries(component_C_node ${PROJECT_NAME})

add_executable(component_D_node src/component_D_node.cpp)
ament_target_dependencies(component_D_node ${dependencies})
target_link_libraries(component_D_node ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  component_A_node
  component_B_node
  component_C_node
  component_D_node
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
