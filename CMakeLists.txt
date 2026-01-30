cmake_minimum_required(VERSION 3.8)
project(ros_fuelgauge_max17263)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(max17263_node src/max17263.cpp
                        lib/libi2c/src/i2c.c
                        lib/driver/src/max1726x.c)
ament_target_dependencies(max17263_node rclcpp sensor_msgs)
target_include_directories(max17263_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/max17263
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libi2c/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/driver/include)
target_compile_features(max17263_node PUBLIC c_std_99 cxx_std_20)

# target_link_libraries(max17263_node "${cpp_typesupport_target}")

install(TARGETS max17263_node
  DESTINATION lib/${PROJECT_NAME})

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
