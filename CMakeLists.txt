cmake_minimum_required(VERSION 3.8)
project(matrix_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
     "srv/MultiplyTwoFloats.srv"
     DEPENDENCIES std_msgs
   )

# Add executable
add_executable(matrix_publisher_node src/matrix_publisher_node.cpp)
ament_target_dependencies(matrix_publisher_node rclcpp std_msgs)
add_executable(matrix_subscriber_node src/matrix_subscriber_node.cpp)
ament_target_dependencies(matrix_subscriber_node rclcpp std_msgs)
add_executable(add_server_node src/add_server_node.cpp)
ament_target_dependencies(add_server_node rclcpp example_interfaces)
add_executable(adder_client_node src/adder_client_node.cpp)
ament_target_dependencies(adder_client_node rclcpp example_interfaces)
add_executable(interactive_client_node src/interactive_client_node.cpp)
ament_target_dependencies(interactive_client_node rclcpp example_interfaces)
add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node rclcpp std_msgs example_interfaces)
rosidl_target_interfaces(bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(multiply_server_node src/multiply_server_node.cpp)
ament_target_dependencies(multiply_server_node rclcpp)
rosidl_target_interfaces(multiply_server_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(laser_listener_node src/laser_listener_node.cpp)
ament_target_dependencies(laser_listener_node rclcpp sensor_msgs)
add_executable(obstacle_avoider_node src/obstacle_avoider_node.cpp)
ament_target_dependencies(obstacle_avoider_node rclcpp sensor_msgs geometry_msgs)
add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher rclcpp geometry_msgs)
add_executable(teleop_control_node src/teleop_control_node.cpp)
ament_target_dependencies(teleop_control_node sensor_msgs visualization_msgs rclcpp geometry_msgs)
add_executable(wall_follower_node src/wall_follower_node.cpp)
ament_target_dependencies(wall_follower_node sensor_msgs visualization_msgs rclcpp geometry_msgs)

# Install executable
install(TARGETS matrix_publisher_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS matrix_subscriber_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS add_server_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS adder_client_node
  DESTINATION lib/${PROJECT_NAME}) 
install(TARGETS interactive_client_node 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS multiply_server_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS laser_listener_node 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS obstacle_avoider_node 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cmd_vel_publisher 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS teleop_control_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS wall_follower_node

  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
