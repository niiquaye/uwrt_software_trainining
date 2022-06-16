cmake_minimum_required(VERSION 3.5)
project(software_training_assignment)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#include the 'include' directory
include_directories(include)

# add clear_turtles_node
add_library(turtle_request SHARED 
          src/clear_turtles_node.cpp)
          
target_compile_definitions(turtle_request PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(turtle_request
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs")   
rclcpp_components_register_nodes(turtle_request "composition::turtle_service_request_node")

#add reset_turtle_node
add_library(turtle_reset_request SHARED
            src/reset_turtle_node.cpp)

ament_target_dependencies(turtle_reset_request
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs")   
rclcpp_components_register_nodes(turtle_reset_request "composition::turtle_service_request_node")


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Software.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)


add_library(circular_turtle_request SHARED
            src/circular_turtle_node.cpp)
ament_target_dependencies(circular_turtle_request
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs")
rclcpp_components_register_nodes(circular_turtle_request "composition::circular_turtle_node")


add_library(spawn_turtle_request SHARED
            src/spawn_turtle_node.cpp)
ament_target_dependencies(spawn_turtle_request
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs")
rclcpp_components_register_nodes(circular_turtle_request "composition::spawn_turtle_node")


add_library(turtle_distance_request SHARED
            src/turtle_distance_node.cpp)
ament_target_dependencies(turtle_distance_request
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs")
rclcpp_components_register_nodes(turtle_distance_request "composition::turtle_distance_node")




install(TARGETS
  turtle_request
  turtle_reset_request
  circular_turtle_request
  turtle_distance_request
  spawn_turtle_request
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
