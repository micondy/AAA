cmake_minimum_required(VERSION 3.8)
project(mobile_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config lattice launch web
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/carter_mapping_route.py
    scripts/collision_safety_benchmark.py
    scripts/cmd_vel_priority_mux.py
    scripts/four_camera_viewer.py
    scripts/dynamic_obstacle_intrusion_benchmark.py
    scripts/isaac_obstacle_safety_benchmark.py
    scripts/navigation_waypoint_benchmark.py
    scripts/nvblox_costmap_obstacle_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav2_config test/test_nav2_config.py)
endif()

ament_package()
