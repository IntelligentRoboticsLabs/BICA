cmake_minimum_required(VERSION 2.8.3)
project(bica_examples)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  bica
  bica_graph
  roscpp
  rospy
  std_msgs
  bica_graph
  bica_planning
  bica_msgs
  image_transport
  cv_bridge
  pcl_ros
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

add_executable(graph_client_A_node src/graph/graph_client_A_node.cpp)
add_dependencies(graph_client_A_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_client_A_node ${catkin_LIBRARIES})

add_executable(graph_client_B_node src/graph/graph_client_B_node.cpp)
add_dependencies(graph_client_B_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_client_B_node ${catkin_LIBRARIES})


# FollowBall Example ####################

add_executable(ball_follower_node src/graph/follow_ball/ball_follower_node.cpp)
add_dependencies(ball_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_follower_node ${catkin_LIBRARIES})

add_executable(ball_detector_node src/graph/follow_ball/ball_detector_node.cpp)
add_dependencies(ball_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_detector_node ${catkin_LIBRARIES})

# Follow Ball&Nets Example ####################

add_executable(ballnet_follower_node
  src/graph/follow_ballnet/ballnet_follower_node.cpp
  src/graph/follow_ballnet/ball_net_follower.cpp)
add_dependencies(ballnet_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ballnet_follower_node ${catkin_LIBRARIES})

add_executable(yellow_net_detector_node src/graph/follow_ballnet/yellow_net_detector_node.cpp)
add_dependencies(yellow_net_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yellow_net_detector_node ${catkin_LIBRARIES})

add_executable(blue_net_detector_node src/graph/follow_ballnet/blue_net_detector_node.cpp)
add_dependencies(blue_net_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blue_net_detector_node ${catkin_LIBRARIES})

# Follow Ball&Nets Global Example ####################

add_executable(ballnet_global_follower_node
  src/graph/follow_global_ballnet/ballnet_global_follower_node.cpp
  src/graph/follow_global_ballnet/ball_net_follower.cpp)
add_dependencies(ballnet_global_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ballnet_global_follower_node ${catkin_LIBRARIES})

add_executable(ball_global_detector_node src/graph/follow_global_ballnet/ball_global_detector_node.cpp)
add_dependencies(ball_global_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_global_detector_node ${catkin_LIBRARIES})

add_executable(yellow_net_global_detector_node src/graph/follow_global_ballnet/yellow_net_global_detector_node.cpp)
add_dependencies(yellow_net_global_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yellow_net_global_detector_node ${catkin_LIBRARIES})

add_executable(blue_net_global_detector_node src/graph/follow_global_ballnet/blue_net_global_detector_node.cpp)
add_dependencies(blue_net_global_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blue_net_global_detector_node ${catkin_LIBRARIES})

# Planning Example ####################
add_executable(planning_example_node src/planning/planning_example_node.cpp)
add_dependencies(planning_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_example_node ${catkin_LIBRARIES})

add_executable(approach_action_node src/planning/hri/Approach.cpp)
add_dependencies(approach_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(approach_action_node ${catkin_LIBRARIES})

add_executable(talk_action_node src/planning/hri/Talk.cpp)
add_dependencies(talk_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(talk_action_node ${catkin_LIBRARIES})

add_executable(move_action_node src/planning/navigate/Move.cpp)
add_dependencies(move_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_action_node ${catkin_LIBRARIES})
