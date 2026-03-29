cmake_minimum_required(VERSION 3.8)
project(dbvf_autonomy)

find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Messages/services are in the separate dbvf_msgs package to avoid
# namespace collision between rosidl-generated code and this Python package.

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install node executables
install(PROGRAMS
  scripts/mavlink_interface_node
  scripts/tag_detector_adapter_node
  scripts/precision_landing_node
  scripts/tag_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mavlink test/test_mavlink_messages.py)
  ament_add_pytest_test(test_debounce test/test_debounce_filter.py)
  ament_add_pytest_test(test_angles test/test_angle_computation.py)
  ament_add_pytest_test(test_tags test/test_tag_selection.py)
  ament_add_pytest_test(test_fsm test/test_state_machine.py)
  ament_add_pytest_test(test_pid test/test_pid_controller.py)
  ament_add_pytest_test(test_pose test/test_pose_transform.py)
  ament_add_pytest_test(test_tag_pose_est test/test_tag_pose_estimation.py)
  ament_add_pytest_test(test_descent_clamp test/test_descent_rate_clamp.py)
  ament_add_pytest_test(test_mission_helpers test/test_mission_helpers.py)
endif()

ament_package()
