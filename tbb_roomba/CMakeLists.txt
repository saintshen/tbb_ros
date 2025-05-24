cmake_minimum_required(VERSION 3.8)
project(tbb_roomba)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(tbb_roomba_node src/tbb_roomba_node.cpp)
ament_target_dependencies(tbb_roomba_node rclcpp) 

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  tbb_roomba_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
