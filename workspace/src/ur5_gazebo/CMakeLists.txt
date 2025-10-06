cmake_minimum_required(VERSION 3.8)
project(ur5_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install Python executables to create libexec dir
install(PROGRAMS
  scripts/camera_detector.py
  scripts/pick_place_controller.py
  scripts/demo_recorder.py
  scripts/bc_policy_controller.py
  scripts/comparison_evaluator.py
  scripts/environment_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()