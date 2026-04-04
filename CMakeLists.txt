cmake_minimum_required(VERSION 3.8)
project(drempower_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
)

# Include directories
include_directories(include)

# Create driver library
add_library(drempower_driver src/drempower_driver.cpp)
target_include_directories(drempower_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(drempower_driver PUBLIC Boost::system)

# Create the node executable
add_executable(drempower_node src/drempower_node.cpp)
target_link_libraries(drempower_node drempower_driver)
ament_target_dependencies(drempower_node rclcpp sensor_msgs std_msgs)
rosidl_target_interfaces(drempower_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables and scripts
install(TARGETS drempower_node drempower_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/test_sdk_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch include
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Important: ensure the driver library and the node can find the generated headers
add_dependencies(drempower_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

ament_package()
