cmake_minimum_required(VERSION 3.8)
project(mobile_base_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_damiao_calibration test/test_damiao_calibration.py)
  ament_add_pytest_test(test_chassis_launch_config test/test_chassis_launch_config.py)
endif()

install(DIRECTORY config docs launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/calibrate_damiao_steering.py
  scripts/configure_damiao_mode.py
  scripts/setup_can.sh
  scripts/check_two_host_network.sh
  scripts/test_zlac8015d.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
