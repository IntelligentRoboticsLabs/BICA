cmake_minimum_required(VERSION 3.5)
project(bica_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ActivateComponent.srv" 
  "srv/DeactivateComponent.srv" 
  "msg/GraphUpdate.msg" 
 DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
