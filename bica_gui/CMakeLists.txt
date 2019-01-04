cmake_minimum_required(VERSION 2.8.3)
project(bica_gui)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)

catkin_python_setup()

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references,-whitespace/braces")
roslint_cpp()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
)


set (bica_gui_SRCS
  src/bica_gui/vicode.cpp
  src/bica_gui/HFSMScene.cpp
  src/bica_gui/State.cpp
  src/bica_gui/Transition.cpp
  src/bica_gui/Component.cpp
  src/bica_gui/Dependency.cpp
)

set(bica_gui_HDRS
  include/bica_gui/vicode.h
  include/bica_gui/HFSMScene.h
  include/bica_gui/State.h
  include/bica_gui/Transition.h
  include/bica_gui/Component.h
  include/bica_gui/Dependency.h
)

set(bica_gui_UIS
  src/bica_gui/vicode.ui
)

qt5_wrap_cpp(bica_gui_MOCS ${bica_gui_HDRS})
qt5_wrap_ui(bica_gui_UIS_H ${bica_gui_UIS})

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${bica_gui_SRCS}
    ${bica_gui_MOCS}
    ${bica_gui_UIS_H}
    )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Xml
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

roslint_cpp(
   src/bica_gui/Component.cpp include/bica_gui/Component.h
   src/bica_gui/Dependency.cpp include/bica_gui/Dependency.h
   src/bica_gui/State.cpp include/bica_gui/State.h
   src/bica_gui/Transition.cpp include/bica_gui/Transition.h
   src/bica_gui/HFSM.cpp include/bica_gui/HFSM.h
   src/bica_gui/HFSMScene.cpp include/bica_gui/HFSMScene.h
   src/bica_gui/vicode.cpp include/bica_gui/vicode.h
)


install(PROGRAMS scripts/bica_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
