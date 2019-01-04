cmake_minimum_required(VERSION 2.8.3)
project(bica_examples)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  bica
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_A src/nodes_cpp/node_A.cpp)
add_dependencies(node_A ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_A ${catkin_LIBRARIES})

add_executable(node_B src/nodes_cpp/node_B.cpp)
add_dependencies(node_B ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_B ${catkin_LIBRARIES})

add_executable(node_C src/nodes_cpp/node_C.cpp)
add_dependencies(node_C ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_C ${catkin_LIBRARIES})

add_executable(node_D src/nodes_cpp/node_D.cpp)
add_dependencies(node_D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_D ${catkin_LIBRARIES})

add_executable(node_M
  src/HFSM/test_M.cpp
  src/HFSM/test_M_impl.cpp
  )
add_dependencies(node_M ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_M ${catkin_LIBRARIES})
