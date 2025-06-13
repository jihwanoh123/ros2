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
