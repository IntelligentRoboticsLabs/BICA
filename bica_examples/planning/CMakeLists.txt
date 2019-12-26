add_executable(node_A_planning components/node_A_planning.cpp)
ament_target_dependencies(node_A_planning ${dependencies})

add_executable(node_B_planning components/node_B_planning.cpp)
ament_target_dependencies(node_B_planning ${dependencies})

add_executable(node_C_planning components/node_C_planning.cpp)
ament_target_dependencies(node_C_planning ${dependencies})

add_executable(node_D_planning components/node_D_planning.cpp)
ament_target_dependencies(node_D_planning ${dependencies})


add_executable(move_action_node actions/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(patrol_action_node actions/patrol_action_node.cpp)
ament_target_dependencies(patrol_action_node ${dependencies})

add_executable(bica_patrolling_controller_node bica_patrolling_controller_node.cpp)
ament_target_dependencies(bica_patrolling_controller_node ${dependencies})


install(DIRECTORY pddl params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  node_A_planning
  node_B_planning
  node_C_planning
  node_D_planning
  move_action_node
  patrol_action_node
  bica_patrolling_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
