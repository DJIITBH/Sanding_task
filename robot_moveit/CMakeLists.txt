cmake_minimum_required(VERSION 3.8)
project(robot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# cpp api of moveit
find_package(custom_service REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

ament_auto_find_build_dependencies()

include_directories(${custom_service_INCLUDE_DIRS})

ament_auto_add_executable(simple_moveit_interface 
  src/simple_moveit_interface.cpp)
# Install the executable
install(TARGETS simple_moveit_interface
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pos_goal src/pos_goal.cpp)
ament_target_dependencies(pos_goal rclcpp moveit_ros_planning_interface trajectory_msgs builtin_interfaces  visualization_msgs)

install(TARGETS pos_goal
  DESTINATION lib/${PROJECT_NAME}
)

add_library(action_server SHARED src/action_server.cpp)
target_include_directories(action_server  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_server
PRIVATE "ACTION_SERVER_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_server
  "custom_service"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
rclcpp_components_register_node(action_server
PLUGIN "robot_moveit::ActionServer"
EXECUTABLE action_server_node
)

# action client
add_library(action_client SHARED src/action_client.cpp)
target_include_directories(action_client  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_client
PRIVATE "ACTION_CLIENT_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_client
  "custom_service"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
)
rclcpp_components_register_node(action_client
PLUGIN "robot_moveit::ActionClient"
EXECUTABLE action_client_node
)

# task server
add_library(task_server SHARED src/task_server.cpp)
target_include_directories(task_server  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(task_server
PRIVATE "TASK_SERVER_CPP_BUILDING_DLL"
)
ament_target_dependencies(task_server
  "custom_service"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
)
rclcpp_components_register_node(task_server
PLUGIN "robot_moveit::TaskServer"
EXECUTABLE task_server_node
)

# robot_server
add_library(robot_server SHARED src/robot_server.cpp)
target_include_directories(robot_server  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(robot_server
PRIVATE "ROBOT_SERVER_CPP_BUILDING_DLL"
)
ament_target_dependencies(robot_server
  "custom_service"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "visualization_msgs"
  "geometry_msgs"
)
rclcpp_components_register_node(robot_server
PLUGIN "robot_moveit::RobotServer"
EXECUTABLE robot_server_node
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
 action_server
 action_client
 task_server
 robot_server
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin

)

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
