cmake_minimum_required(VERSION 3.5)
project(assignment2_rt_part2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Python client library for ROS2
find_package(std_msgs REQUIRED)  # Standard message types
find_package(rosidl_default_generators REQUIRED)  # For generating interfaces (services/messages)

# Setup for Python package
find_package(ament_python REQUIRED)

# Generate service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLinearVelocity.srv" # Service file location
  DEPENDENCIES std_msgs  # Dependencies for your service file (e.g., std_msgs)
)

# Install Python scripts and libraries
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament package setup
ament_package()
