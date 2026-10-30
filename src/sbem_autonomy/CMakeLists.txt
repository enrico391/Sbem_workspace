cmake_minimum_required(VERSION 3.8)
project(sbem_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(opennav_docking_bt REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(autonomy_node src/autonomy_node.cpp src/bt_nodes.cpp)
target_compile_features(autonomy_node PUBLIC cxx_std_17)
target_include_directories(autonomy_node PRIVATE include)
ament_target_dependencies(autonomy_node
  rclcpp rclcpp_action sensor_msgs nav2_behavior_tree behaviortree_cpp_v3
  ament_index_cpp action_msgs std_msgs std_srvs)

install(TARGETS autonomy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_battery_policy test/test_battery_policy.cpp)
  target_include_directories(test_battery_policy PRIVATE include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_autonomy test/test_autonomy.py TIMEOUT 90
    ENV ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1)
endif()

ament_package()
