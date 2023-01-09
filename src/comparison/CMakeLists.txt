cmake_minimum_required(VERSION 3.8)
project(comparison)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(zenohc REQUIRED)
find_package(zenohcpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
)

add_library(${PROJECT_NAME}_lib INTERFACE)
target_include_directories(${PROJECT_NAME}_lib
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME}_lib INTERFACE zenohc::lib zenohcpp)
ament_target_dependencies(${PROJECT_NAME}_lib INTERFACE ${DEPENDENCIES})

# # include_directories("include")

add_executable(ros_sub src/frame_rate_counter/ros_sub.cpp)
add_executable(zenoh_sub src/frame_rate_counter/zenoh_sub.cpp)
target_link_libraries(ros_sub ${PROJECT_NAME}_lib)
target_link_libraries(zenoh_sub ${PROJECT_NAME}_lib)

# ament_target_dependencies(ros_sub ${DEPENDENCIES})
# target_link_libraries(zenoh_sub zenohc::lib zenohcpp)

# add_executable(ros_pub src/transfer/ros_pub.cpp)
# add_executable(zenoh_pub src/transfer/zenoh_pub.cpp)
# add_executable(sub_node src/frame_rate_counter/sub_node.cpp)
# add_executable(zenoh_sub src/frame_rate_counter/zenoh_sub.cpp)

# target_link_libraries(zenoh_sub zenohc::lib zenohcpp)
# target_link_libraries(zenoh_pub zenohc::lib zenohcpp)
# target_link_libraries(sub_node ros_sub_lib)

# ament_target_dependencies(ros_pub rclcpp sensor_msgs)
# ament_target_dependencies(ros_sub_lib rclcpp sensor_msgs)
# ament_target_dependencies(sub_node rclcpp sensor_msgs)
# ament_target_dependencies(zenoh_pub rclcpp sensor_msgs)
# ament_target_dependencies(zenoh_sub rclcpp sensor_msgs)

install(TARGETS ros_sub
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS sub_node ros_pub zenoh_pub zenoh_sub
#   DESTINATION lib/${PROJECT_NAME})

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
