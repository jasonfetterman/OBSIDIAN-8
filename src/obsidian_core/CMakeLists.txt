cmake_minimum_required(VERSION 3.8)
project(obsidian_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(reactive_core src/reactive_core.cpp)
ament_target_dependencies(reactive_core rclcpp std_msgs)

# Install executable
install(TARGETS
  reactive_core
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
