cmake_minimum_required(VERSION 3.8)
project(webots_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(webots_ros2_driver REQUIRED)

pluginlib_export_plugin_description_file(webots_ros2_driver webots_driver.xml)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/WebotsDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  jitsuyo
  kansei_interfaces
  keisan
  OpenCV
  pluginlib
  rclcpp
  tachimawari
  tachimawari_interfaces
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
