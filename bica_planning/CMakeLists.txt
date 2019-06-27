cmake_minimum_required(VERSION 2.8.3)
project(bica_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rosplan_knowledge_msgs
	diagnostic_msgs
	rosplan_dispatch_msgs
	rosplan_knowledge_msgs
	rosplan_planning_system
	actionlib
  bica
  roslint
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES bica_planning
	CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs diagnostic_msgs rosplan_planning_system
	DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references,-whitespace/braces,-build/c++11")
roslint_cpp()

add_library(bica_planning
	 src/Executor.cpp
     src/KMSClient.cpp
     src/Action.cpp

 )

add_dependencies(bica_planning ${catkin_EXPORTED_TARGETS})
target_link_libraries(bica_planning ${catkin_LIBRARIES})

install(TARGETS bica_planning bica_planning
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
install(DIRECTORY pddl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pddl/
)
install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
