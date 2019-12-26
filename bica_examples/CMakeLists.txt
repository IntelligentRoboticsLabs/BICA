cmake_minimum_required(VERSION 3.5)
project(bica_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bica REQUIRED)
find_package(bica_graph REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    bica
    bica_graph
    plansys2_executor
    plansys2_domain_expert
    lifecycle_msgs
    nav2_msgs
)

add_subdirectory(nodes_cpp)
add_subdirectory(graph)
add_subdirectory(planning)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
