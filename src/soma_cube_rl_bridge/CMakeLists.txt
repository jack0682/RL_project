cmake_minimum_required(VERSION 3.8)
project(soma_cube_rl_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dsr_msgs2 REQUIRED)
find_package(dsr_common2 REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RLReset.srv"
  "srv/RLStep.srv"
  "srv/GetSafetyState.srv"
  "msg/RLObservation.msg"
  "msg/SafetyState.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Include directories
include_directories(include)

# Add executable for motion proxy node
add_executable(motion_proxy_node src/motion_proxy_node.cpp)
ament_target_dependencies(motion_proxy_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  dsr_msgs2
  dsr_common2
)

# Add executable for safety monitor node  
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node 
  rclcpp 
  std_msgs 
  sensor_msgs
  dsr_msgs2
  dsr_common2
)

# Add executable for RL environment node
add_executable(rl_env_node src/rl_env_node.cpp)
ament_target_dependencies(rl_env_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  dsr_msgs2
  dsr_common2
)

# Link against generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(motion_proxy_node "${cpp_typesupport_target}")
target_link_libraries(safety_monitor_node "${cpp_typesupport_target}")
target_link_libraries(rl_env_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  motion_proxy_node
  safety_monitor_node
  rl_env_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install training system
install(DIRECTORY
  training_system/
  DESTINATION share/${PROJECT_NAME}/training_system
  USE_SOURCE_PERMISSIONS
)

# Install header files
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()