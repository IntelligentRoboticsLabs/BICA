add_executable(node_A node_A.cpp)
ament_target_dependencies(node_A ${dependencies})

add_executable(node_B node_B.cpp)
ament_target_dependencies(node_B ${dependencies})

add_executable(node_C node_C.cpp)
ament_target_dependencies(node_C ${dependencies})

add_executable(node_D node_D.cpp)
ament_target_dependencies(node_D ${dependencies})

install(TARGETS
  node_A
  node_B
  node_C
  node_D
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
