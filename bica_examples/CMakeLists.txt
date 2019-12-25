cmake_minimum_required(VERSION 3.5)
project(bica_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bica REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    bica
)

add_subdirectory(nodes_cpp)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
