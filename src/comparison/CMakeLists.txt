cmake_minimum_required(VERSION 3.8)
project(comparison)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# include_directories("include")
# comparison common interface
add_library(${PROJECT_NAME}_lib INTERFACE)
target_include_directories(${PROJECT_NAME}_lib
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
ament_target_dependencies(${PROJECT_NAME}_lib INTERFACE rclcpp sensor_msgs)

# link libraries for all targets below
link_libraries(${PROJECT_NAME}_lib)

add_executable(ros_pub       src/transfer/ros_pub.cpp)
add_executable(ros_sub       src/frame_rate_counter/ros_sub.cpp)
add_executable(ros_pcd_gen   src/pcd_generator/ros_pcd_gen.cpp)

install(
  TARGETS
    ros_pub
    ros_sub
    ros_pcd_gen
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
