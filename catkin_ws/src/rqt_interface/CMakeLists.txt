cmake_minimum_required(VERSION 3.0)
project(rqt_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  interactive_markers
  visualization_msgs
  geometry_msgs
  tf
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rqt_interface
  src/main.cpp
  src/main_window.cpp
  src/rviz_panel.cpp
  src/pipeline_controller.cpp

  include/main_window.hpp
  include/rviz_panel.hpp
  include/pipeline_controller.hpp
)

add_executable(ee_marker_node
  src/ee_marker_node.cpp
)

add_executable(rubiks_cube_marker_node
  src/rubiks_cube_marker_node.cpp
)

target_link_libraries(rqt_interface
  Qt5::Widgets
  ${catkin_LIBRARIES}
  yaml-cpp
)

target_link_libraries(ee_marker_node
  ${catkin_LIBRARIES}
)

target_link_libraries(rubiks_cube_marker_node
  ${catkin_LIBRARIES}
)

add_dependencies(ee_marker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(rubiks_cube_marker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)