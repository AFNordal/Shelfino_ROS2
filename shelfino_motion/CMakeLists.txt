cmake_minimum_required(VERSION 3.8)
project(shelfino_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(example_interfaces REQUIRED)
#find_package(rclcpp_action REQUIRED)
#find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Link additional libraries

# Include headers
include_directories(include)


add_library(motion_controller src/motion_controller.cpp)
ament_target_dependencies(motion_controller rclcpp geometry_msgs nav_msgs tf2_geometry_msgs tf2)



# Create executable
add_executable(motion_node src/motion_controller.cpp)
target_link_libraries(motion_node motion_controller)
ament_target_dependencies(motion_node rclcpp geometry_msgs)

# Install targets
install(TARGETS motion_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
