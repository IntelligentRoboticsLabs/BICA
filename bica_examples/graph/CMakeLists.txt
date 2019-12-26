add_executable(node_A_graph node_A_graph.cpp)
ament_target_dependencies(node_A_graph ${dependencies})

add_executable(node_B_graph node_B_graph.cpp)
ament_target_dependencies(node_B_graph ${dependencies})

add_executable(node_C_graph node_C_graph.cpp)
ament_target_dependencies(node_C_graph ${dependencies})

add_executable(node_D_graph node_D_graph.cpp)
ament_target_dependencies(node_D_graph ${dependencies})

install(TARGETS
  node_A_graph
  node_B_graph
  node_C_graph
  node_D_graph
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
