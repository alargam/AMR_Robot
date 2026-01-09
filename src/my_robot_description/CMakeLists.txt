cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

# 1. Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Find dependencies [cite: 2025-10-22]
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 3. Install directories (CRITICAL for SLAM and Launch to work) [cite: 2025-10-22]
install(DIRECTORY 
  launch 
  config 
  urdf 
  mesh
  maps
  DESTINATION share/${PROJECT_NAME}
)

# 4. Testing (Standard ROS 2)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 5. Export package [cite: 2025-10-22]
ament_package()