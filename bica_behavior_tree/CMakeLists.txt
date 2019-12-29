cmake_minimum_required(VERSION 3.5)
project(bica_behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bica REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
    rclcpp
    bica
    ament_index_cpp
    behaviortree_cpp_v3
)

include_directories(include)

add_library(activation_sequence_bt_node SHARED plugins/control/activation_sequence.cpp)
list(APPEND plugin_libs activation_sequence_bt_node)

add_library(activation_action_bt_node SHARED plugins/action/activation_action.cpp)
list(APPEND plugin_libs activation_action_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})

ament_package()
