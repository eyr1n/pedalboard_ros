cmake_minimum_required(VERSION 3.8)
project(junction_box)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pedalboard_msgs REQUIRED)

pkg_check_modules(JACK REQUIRED jack)

add_library(${PROJECT_NAME}_component SHARED src/junction_box.cpp)
target_link_libraries(${PROJECT_NAME}_component ${JACK_LIBRARIES} Boost::boost)
target_include_directories(${PROJECT_NAME}_component PRIVATE ${JACK_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "pedalboard::JunctionBox"
  EXECUTABLE ${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME}_component HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
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
