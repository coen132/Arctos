cmake_minimum_required(VERSION 3.22)
project(arctos_description)

find_package(ament_cmake REQUIRED)
find_package(arctos_hardware_interface REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
ament_package()



