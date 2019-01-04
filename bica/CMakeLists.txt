cmake_minimum_required(VERSION 2.8.3)
project(bica)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  roslint
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bica
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
)

add_library(bica
   src/Component.cpp
   src/Activation.cpp
   src/Dependency.cpp
)

add_executable(launcher src/launcher.cpp)
target_link_libraries(launcher ${LINK_LIBS} bica)

roslint_cpp(
   src/Component.cpp include/bica/Component.h
   src/Activation.cpp include/bica/Activation.h
   src/Dependency.cpp include/bica/Dependency.h
   src/launcher.cpp
   include/bica/HfsmComponent.h
)


install(TARGETS bica
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
