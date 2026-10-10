cmake_minimum_required(VERSION 3.20)
project(lopsy_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lopsy_interfaces REQUIRED)

add_library(tendon_kinematics STATIC src/tendon_kinematics.cpp)
target_include_directories(tendon_kinematics PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(lopsy_state_publisher src/lopsy_state_publisher.cpp)
target_link_libraries(lopsy_state_publisher PRIVATE rclcpp::rclcpp tendon_kinematics ${lopsy_interfaces_TARGETS} ${sensor_msgs_TARGETS})

add_executable(tendon_force_publisher src/tendon_force_publisher.cpp)
target_link_libraries(tendon_force_publisher PRIVATE rclcpp::rclcpp ${lopsy_interfaces_TARGETS})

install(TARGETS
  lopsy_state_publisher
  tendon_force_publisher
  DESTINATION lib/${PROJECT_NAME}
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
