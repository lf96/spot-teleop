cmake_minimum_required(VERSION 3.0)
project(rqt_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
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

target_link_libraries(rqt_interface
  Qt5::Widgets
  ${catkin_LIBRARIES}
  yaml-cpp
)
