cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gbplanner3_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs gbplanner3_interfaces nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs gbplanner3_interfaces nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS scripts/keypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_rolling_height test/test_rolling_height.cpp)
  target_include_directories(test_rolling_height PRIVATE include)
  ament_add_gtest(test_exploration_intent test/test_exploration_intent.cpp)
  target_include_directories(test_exploration_intent PRIVATE include)
  target_link_libraries(test_exploration_intent Eigen3::Eigen)
  add_launch_test(test/test_exploration_mode.py TIMEOUT 30)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_contract_planning.py TIMEOUT 30)
  add_launch_test(test/test_contract_live.py TIMEOUT 30)
  add_launch_test(test/test_controller_contract.py TIMEOUT 30)
  add_launch_test(test/test_terminal_controller.py TIMEOUT 30)
  ament_add_gtest(test_short_optimization test/test_short_optimization.cpp)
  target_include_directories(test_short_optimization PRIVATE include)
  ament_target_dependencies(test_short_optimization bspline_opt traj_utils plan_env path_searching rclcpp)
  target_link_libraries(test_short_optimization Eigen3::Eigen)
  ament_add_gtest(test_short_initialization test/test_short_initialization.cpp)
  ament_target_dependencies(test_short_initialization bspline_opt traj_utils)
  target_link_libraries(test_short_initialization Eigen3::Eigen)
  ament_add_gtest(test_contract_handoff test/test_contract_handoff.cpp
    src/scan_replan_fsm.cpp src/planner_manager.cpp)
  target_include_directories(test_contract_handoff PRIVATE include)
  ament_target_dependencies(test_contract_handoff ${scan_planner_dependencies})
  target_link_libraries(test_contract_handoff Eigen3::Eigen)
  ament_add_gtest(test_goal_feedback test/test_goal_feedback.cpp)
  target_include_directories(test_goal_feedback PRIVATE include)
  ament_target_dependencies(test_goal_feedback geometry_msgs)
  ament_add_gtest(test_terminal_speed test/test_terminal_speed.cpp)
  target_include_directories(test_terminal_speed PRIVATE include)
  ament_add_gtest(test_reference_path test/test_reference_path.cpp)
  target_include_directories(test_reference_path PRIVATE include)
  ament_target_dependencies(test_reference_path traj_utils)
  target_link_libraries(test_reference_path Eigen3::Eigen)
  ament_add_gtest(test_reference_interval test/test_reference_interval.cpp)
  target_include_directories(test_reference_interval PRIVATE include)
  ament_target_dependencies(test_reference_interval bspline_opt traj_utils rclcpp)
  target_link_libraries(test_reference_interval Eigen3::Eigen)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()

ament_package()
