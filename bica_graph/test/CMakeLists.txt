ament_add_gtest(bica_graph_test bica_graph_test.cpp)
target_link_libraries(bica_graph_test ${PROJECT_NAME})


add_executable(test_graph_A_node test_graph_A_node.cpp)
ament_target_dependencies(test_graph_A_node ${dependencies})
target_link_libraries(test_graph_A_node ${PROJECT_NAME})

add_executable(test_graph_B_node test_graph_B_node.cpp)
ament_target_dependencies(test_graph_B_node ${dependencies})
target_link_libraries(test_graph_B_node ${PROJECT_NAME})

add_executable(test_graph_C_node test_graph_C_node.cpp)
ament_target_dependencies(test_graph_C_node ${dependencies})
target_link_libraries(test_graph_C_node ${PROJECT_NAME})