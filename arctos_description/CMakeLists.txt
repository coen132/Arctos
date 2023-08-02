cmake_minimum_required(VERSION 3.22)
project(arctos_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

