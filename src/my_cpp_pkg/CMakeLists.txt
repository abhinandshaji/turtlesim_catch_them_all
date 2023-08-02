cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

#default to C++ 14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(turtlesim)
find_package(geometry_msgs)
find_package(std_srvs)

add_executable(turtle_client_node src/turtle_client_node.cpp)
ament_target_dependencies(turtle_client_node rclcpp turtlesim)


add_executable(turtle_control_node src/turtle_control_node.cpp)
ament_target_dependencies(turtle_control_node rclcpp turtlesim geometry_msgs std_msgs)

add_executable(turtle_kill_client_node src/turtle_kill_client_node.cpp)
ament_target_dependencies(turtle_kill_client_node rclcpp turtlesim std_msgs std_srvs)


install(TARGETS                                #Adding what all nodes to be installed an where(in python this is done in Setup.cfg and setup.py)
  cpp_node                                     #this is name of executable, node name is what we defined in the executavle file
  robot_news_station                           #name of executable as we defined earlier in the add executable
  smart_phone
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client_no_OOP
  add_two_ints_client
  hw_status_publisher
  hw_status_server
  led_panel_server_node
  battery_client_node
  turtle_client_node
  turtle_control_node
  turtle_kill_client_node
  DESTINATION lib/${PROJECT_NAME}              #install in the install/liv/my_cpp_pkg
)


ament_package()
