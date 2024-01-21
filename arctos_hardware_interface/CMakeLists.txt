cmake_minimum_required(VERSION 3.8)
project(arctos_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rcpputils REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  include
  DESTINATION share/${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/arctos_interface.cpp
  controller/arctos_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include)

ament_target_dependencies(
  ${PROJECT_NAME} 
  can_msgs
  sensor_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  rcpputils
  trajectory_msgs)

target_compile_definitions(${PROJECT_NAME}  PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
target_compile_definitions(${PROJECT_NAME}  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUCNTIONS")
pluginlib_export_plugin_description_file(hardware_interface arctos_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)
install(
  DIRECTORY include/ 
  DESTINATION include
)

install(
  DIRECTORY ros2_control/
  DESTINATION share/arctos_hardware_interface/ros2_control
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
