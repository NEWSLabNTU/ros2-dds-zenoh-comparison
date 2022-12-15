cmake_minimum_required(VERSION 3.8)
project(comparison)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ros_pub src/transfer/ros_pub.cpp)
add_executable(zenoh_pub src/transfer/zenoh_pub.cpp)
add_executable(zenoh_sub src/frame_rate_counter/zenoh_sub.cpp)

# zenohc dependency
find_path(ZENOHC_INCLUDE_DIRS zenohc/zenoh.h)
include_directories(${ZENOHC_INCLUDE_DIRS})
find_library(ZENOHC_LIBRARY zenohc)
target_link_libraries(zenoh_pub ${ZENOHC_LIBRARY})
target_link_libraries(zenoh_sub ${ZENOHC_LIBRARY})

ament_target_dependencies(ros_pub rclcpp sensor_msgs)
ament_target_dependencies(zenoh_pub rclcpp sensor_msgs)
ament_target_dependencies(zenoh_sub rclcpp sensor_msgs)
target_compile_features(ros_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(zenoh_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(zenoh_sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# target_include_directories(ros_pub
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )


install(TARGETS ros_pub zenoh_pub zenoh_sub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

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
