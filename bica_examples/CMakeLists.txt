cmake_minimum_required(VERSION 3.5)
project(bica_examples)

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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bica_behavior_tree REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    bica
    bica_graph
    plansys2_executor
    plansys2_domain_expert
    lifecycle_msgs
    nav2_msgs
    behaviortree_cpp_v3
    ament_index_cpp
    bica_behavior_tree
)

add_subdirectory(nodes_cpp)
add_subdirectory(graph)
add_subdirectory(planning)
add_subdirectory(nodes_bt)

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
