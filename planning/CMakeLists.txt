cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CGAL_DATA_DIR "cgal_data")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Qt5)

add_executable(roadmap_gen 
    src/roadmap_gen.cpp 
    include/planning/mapgeometry.cpp
    include/hammersley/hammersley.cpp
    include/planning/graphs.cpp)
ament_target_dependencies(roadmap_gen rclcpp std_msgs geometry_msgs obstacles_msgs urdf)
target_include_directories(roadmap_gen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/python3.10)
target_link_libraries(roadmap_gen 
  CGAL::CGAL 
  CGAL::CGAL_Basic_viewer)
target_compile_features(roadmap_gen PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  roadmap_gen
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
