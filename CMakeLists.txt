cmake_minimum_required(VERSION 3.8)
project(tb3_nav_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)  
find_package(ros2_linkattacher REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(spawn_collision_objects src/spawn_collision_objects.cpp)
ament_target_dependencies(spawn_collision_objects rclcpp rclcpp_action moveit_ros_planning_interface)

add_executable(basic_pick_and_place src/basic_pick_and_place.cpp)
ament_target_dependencies(basic_pick_and_place rclcpp moveit_ros_planning_interface linkattacher_msgs ros2_linkattacher)

add_executable(static_pick_and_place src/static_pick_and_place.cpp)
ament_target_dependencies(static_pick_and_place rclcpp moveit_ros_planning_interface linkattacher_msgs ros2_linkattacher)

add_executable(navigate_to_pose src/nav2_pose.cpp)
ament_target_dependencies(navigate_to_pose rclcpp rclcpp_action nav2_msgs tf2_geometry_msgs)

add_executable(navigate_through_poses src/nav_through_poses.cpp)
ament_target_dependencies(navigate_through_poses rclcpp rclcpp_action nav2_msgs geometry_msgs tf2_geometry_msgs)

install(TARGETS spawn_collision_objects basic_pick_and_place static_pick_and_place navigate_to_pose navigate_through_poses
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch gazebo
  DESTINATION share/${PROJECT_NAME})


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
