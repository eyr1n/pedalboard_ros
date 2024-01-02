cmake_minimum_required(VERSION 3.8)
project(effects)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pedalboard_msgs REQUIRED)

pkg_check_modules(SNDFILE REQUIRED sndfile)
pkg_check_modules(FFTW REQUIRED fftw3f)

# passthrough
add_library(passthrough_component SHARED src/passthrough.cpp)
ament_target_dependencies(passthrough_component
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
rclcpp_components_register_node(passthrough_component
  PLUGIN "pedalboard::Passthrough"
  EXECUTABLE passthrough
)

# distortion
add_library(distortion_component SHARED src/distortion.cpp)
ament_target_dependencies(distortion_component
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
rclcpp_components_register_node(distortion_component
  PLUGIN "pedalboard::Distortion"
  EXECUTABLE distortion
)

# delay
add_library(delay_component SHARED src/delay.cpp)
ament_target_dependencies(delay_component
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
rclcpp_components_register_node(delay_component
  PLUGIN "pedalboard::Delay"
  EXECUTABLE delay
)

# cabinet_sim
add_library(cabinet_sim_component SHARED src/cabinet_sim.cpp)
ament_target_dependencies(cabinet_sim_component
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
target_link_libraries(cabinet_sim_component
    ${SNDFILE_LIBRARIES}
    ${FFTW_LIBRARIES}
)
target_include_directories(cabinet_sim_component PRIVATE
    ${SNDFILE_INCLUDE_DIRS}
    ${FFTW_INCLUDE_DIRS}
)
rclcpp_components_register_node(cabinet_sim_component
  PLUGIN "pedalboard::CabinetSim"
  EXECUTABLE cabinet
)

ament_export_targets(export_${PROJECT_NAME}_component HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  pedalboard_msgs
)
install(
  TARGETS passthrough_component distortion_component delay_component
  EXPORT export_${PROJECT_NAME}_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
