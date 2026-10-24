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

add_library(lopsy_model STATIC
  src/lib/joint_state_mapping.cpp
  src/lib/link_geometry.cpp
  src/lib/mechanics.cpp
  src/lib/model.cpp
  src/lib/statics.cpp)
target_include_directories(lopsy_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(lopsy_state_publisher src/node/lopsy_state_publisher.cpp)
target_link_libraries(lopsy_state_publisher PRIVATE rclcpp::rclcpp lopsy_model ${lopsy_interfaces_TARGETS} ${sensor_msgs_TARGETS})

add_executable(rod_tension_publisher src/node/rod_tension_publisher.cpp)
target_link_libraries(rod_tension_publisher PRIVATE rclcpp::rclcpp ${lopsy_interfaces_TARGETS})

install(TARGETS
  lopsy_state_publisher
  rod_tension_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tension_slider_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tension_slider_gui
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lopsy_model
    test/test_link_geometry.cpp
    test/test_mechanics.cpp
    test/test_statics.cpp
    test/test_kinematics.cpp)
  target_link_libraries(test_lopsy_model lopsy_model)
endif()

ament_package()
