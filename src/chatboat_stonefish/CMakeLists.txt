cmake_minimum_required(VERSION 3.8)
project(chatboat_stonefish)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scenarios data config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
