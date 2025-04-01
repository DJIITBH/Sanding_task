cmake_minimum_required(VERSION 3.8)
project(control_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED )
find_package(ament_index_cpp REQUIRED)
find_package(custom_service REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
# find_package(bt_interfaces REQUIRED)         


ament_auto_find_build_dependencies()


add_executable(force_sub src/force_sub.cpp)
ament_target_dependencies(force_sub rclcpp geometry_msgs)  #declare dependencies of your node

add_executable(bt_node src/behaviour.cpp)
ament_target_dependencies(bt_node rclcpp behaviortree_cpp )

# add_executable(task_control src/task_control.cpp)
# ament_target_dependencies(task_control rclcpp btcpp_ros2_interfaces behaviortree_ros2 behaviortree_cpp rclcpp_action rclcpp_components custom_service)

add_executable(sanding src/sanding.cpp)
ament_target_dependencies(sanding rclcpp behaviortree_cpp btcpp_ros2_interfaces behaviortree_ros2 rclcpp_action custom_service)

target_link_libraries(sanding 
  ${ament_index_cpp_LIBRARIES}
)

install(DIRECTORY behaviour_trees DESTINATION share/${PROJECT_NAME})


install(TARGETS                                             #install dependencies of your target node with destination in lib folder
  force_sub
  bt_node
  sanding
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)


ament_package()
