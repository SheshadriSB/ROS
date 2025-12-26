cmake_minimum_required(VERSION 3.8)
project(scroll_approach_interfaces)

# Default to C++17 (good practice)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)   # if your .srv uses std_msgs types

# Generate interfaces from .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetRealScrollSide.srv"
  DEPENDENCIES std_msgs
)

# Export the generated interfaces so other packages can use them
ament_export_dependencies(rosidl_default_runtime)

# Package configuration
ament_package()