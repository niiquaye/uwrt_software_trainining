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
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)


add_executable(turtle_request src/turtle_neutralize.cpp)
#target_compile_definitions(turtle_request
#    PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(turtle_request
    rclcpp
    rclcpp_components
    turtlesim
    std_msgs)
#rclcpp_components_register_nodes(turtle_request "composition::turtle_service_request_node")
# this way we can execute the component with - ros2 component standalone software_training composition::turtle_service_request_node
#set(node_plugins "${node_plugins}compositon::turtle_service_request_node;$<TARGET_FILE:turtle_request>\n")

add_executable(cmd_vel_publisher src/cmd_vel_moving_turt_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher
    rclcpp
    rclcpp_components
    turtlesim
    geometry_msgs
    std_msgs)

add_executable(turtle_spawn src/spawn_turtle_nodelet.cpp)
ament_target_dependencies(turtle_spawn
    rclcpp
    rclcpp_components
    turtlesim
    std_msgs)

add_executable(turtle_service src/reset_moving_turtle_service.cpp)
ament_target_dependencies(turtle_service
    rclcpp
    rclcpp_components
    turtlesim
    std_msgs)

add_executable(turtle_pub src/turtle_publisher.cpp)
ament_target_dependencies(turtle_pub
    rclcpp
    rclcpp_components
    turtlesim
    std_msgs)

add_library(turtle_action_server SHARED
    src/moving_turtle_action_server.cpp)
target_include_directories(turtle_action_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_definitions(turtle_action_server
    PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(turtle_action_server
    rclcpp
    rclcpp_action
    rclcpp_components
    turtlesim
    std_msgs
    geometry_msgs)
rclcpp_components_register_node(turtle_action_server PLUGIN
    "composition::MovingTurtleActionServer" EXECUTABLE moving_turtle_action_server)

install(TARGETS
    turtle_request
    cmd_vel_publisher
    turtle_spawn
    turtle_service
    turtle_pub
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    turtle_action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
    )

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Software.srv"
    "msg/Software.msg"
    "action/Software.action"
    DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_target_interfaces(turtle_pub
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(turtle_service
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(turtle_action_server
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
