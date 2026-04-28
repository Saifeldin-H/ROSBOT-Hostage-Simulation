cmake_minimum_required(VERSION 3.8)
project(hostage_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_library(hostage_actor_controller SHARED
  src/hostage_actor_controller.cpp
)
target_compile_features(hostage_actor_controller PUBLIC cxx_std_17)
target_link_libraries(hostage_actor_controller
  ignition-gazebo6::ignition-gazebo6
  ignition-msgs8::ignition-msgs8
  ignition-plugin1::ignition-plugin1
  ignition-transport11::ignition-transport11
)

install(TARGETS hostage_actor_controller
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/hostage_mission_node.py
  scripts/hostage_patrol_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
