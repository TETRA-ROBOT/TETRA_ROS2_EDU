cmake_minimum_required(VERSION 3.5)
project(lsc_ros2_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
  include/lsc_ros2_driver
)


if(WIN32)
  link_libraries(ws2_32)
endif()

add_executable(autonics_lsc_lidar
  src/lsc_ros2_driver.cpp
  src/autolidar.cpp
  src/parser.cpp
  src/socket.cpp
)

ament_target_dependencies(autonics_lsc_lidar
  rclcpp
  sensor_msgs
  diagnostic_updater
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and liscense is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  if(NOT WIN32)
    # add_subdirectory(test)
  endif()

endif()

install(TARGETS
  autonics_lsc_lidar
  DESTINATION lib/${PROJECT_NAME}
 )

 install(DIRECTORY
 launch rviz urdf
 DESTINATION share/${PROJECT_NAME}
 )

ament_package()
