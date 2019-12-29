add_executable(node_A_bt node_A_bt.cpp)
ament_target_dependencies(node_A_bt ${dependencies})

add_executable(node_B_bt node_B_bt.cpp)
ament_target_dependencies(node_B_bt ${dependencies})

add_executable(node_C_bt node_C_bt.cpp)
ament_target_dependencies(node_C_bt ${dependencies})

add_executable(node_D_bt node_D_bt.cpp)
ament_target_dependencies(node_D_bt ${dependencies})

install(FILES
  node_A_tree.xml
  node_C_tree.xml
  DESTINATION share/${PROJECT_NAME}/nodes_bt
)

install(TARGETS
  node_A_bt
  node_B_bt
  node_C_bt
  node_D_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
