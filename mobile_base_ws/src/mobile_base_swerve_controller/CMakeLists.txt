cmake_minimum_required(VERSION 3.8)
project(mobile_base_swerve_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_ros REQUIRED)


add_library(swerve_controller_plugin SHARED src/mobile_base_swerve_controller.cpp)
target_include_directories(swerve_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(swerve_controller_plugin
  controller_interface geometry_msgs hardware_interface nav_msgs pluginlib rclcpp realtime_tools tf2_ros)
pluginlib_export_plugin_description_file(controller_interface mobile_base_swerve_controller.xml)
install(TARGETS swerve_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_bounded_motion test/test_bounded_motion.cpp)
  target_include_directories(test_bounded_motion PRIVATE include)
  add_test(NAME bounded_motion COMMAND test_bounded_motion)
endif()

ament_package()
