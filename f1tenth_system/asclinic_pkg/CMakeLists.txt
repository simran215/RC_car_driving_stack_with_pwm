cmake_minimum_required(VERSION 3.5)
project(asclinic_pkg)

# Compile as C++14, which is recommended for ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake and other ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(servo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define a variable for dependencies
set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "servo_msgs"
)

# Include directories
include_directories(
  include
  src/drivers/src/
)

# Add executable and link dependencies
add_executable(asclinic_pkg_node
  src/nodes/template_i2c_internal.cpp
  src/drivers/src/i2c_driver/i2c_driver.cpp
  src/drivers/src/pca9685/pca9685.cpp
)

ament_target_dependencies(asclinic_pkg_node ${DEPENDENCIES})

# Install targets
install(TARGETS asclinic_pkg_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Export include directories and dependencies
ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})


# Specify that this package is an ament package
ament_package()

