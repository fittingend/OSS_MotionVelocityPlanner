cmake_minimum_required(VERSION 3.14)
project(OSS_MotionVelocityPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(motion_velocity_planner
  src/planner.cpp
  src/longitudinal_utils.cpp
  src/smoother.cpp
  src/obstacle_stop.cpp
)

target_include_directories(motion_velocity_planner
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_options(motion_velocity_planner PRIVATE -Wall -Wextra -Wpedantic)

add_executable(main_motion_velocity_planner
  example/main_motion_velocity_planner.cpp
  example/io_utils.cpp
)
target_include_directories(main_motion_velocity_planner
  PRIVATE
    ${PROJECT_SOURCE_DIR}/example
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(main_motion_velocity_planner PRIVATE motion_velocity_planner)

install(TARGETS motion_velocity_planner main_motion_velocity_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
