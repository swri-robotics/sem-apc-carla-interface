cmake_minimum_required(VERSION 3.8)
project(carla_shell_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # Deps list
# set(COMMON_DEPS
#   rclpy
#   rclcpp
#   rclcpp_components
#   carla_msgs  
#   std_msgs
# )

# # Find dependencies
# find_package(ament_cmake REQUIRED
#   ${COMMON_DEPS})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(carla_msgs REQUIRED)

# Add nodes
# add_executable(carla_vehicle_interface src/carla_vehicle_interface.cpp)
# target_include_directories(carla_vehicle_interface PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(carla_vehicle_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Add components
add_library(carla_vehicle_interface SHARED
  src/components/carla_vehicle_interface.cpp
)
target_compile_definitions(carla_vehicle_interface
  PRIVATE "carla_shell_bridge_BUILDING_DLL")
ament_target_dependencies(carla_vehicle_interface
  "rclcpp"
  "rclcpp_components"
  "carla_msgs"

)
rclcpp_components_register_nodes(carla_vehicle_interface "carla_shell_bridge::CarlaSimulationVehicleInterface")

# Install component targets
install(
  TARGETS
  carla_vehicle_interface

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install node targets
# install(TARGETS 
#   carla_vehicle_interface
#   DESTINATION lib/${PROJECT_NAME})

# Install Python nodes
install(
  PROGRAMS
  scripts/map_loader.py
  scripts/traffic_generator.py
  DESTINATION
  lib/${PROJECT_NAME}
)

# Install launch directory
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


file(GLOB YAML_CONFIG_FILES config/*.yaml)
install(
  FILES
    ${YAML_CONFIG_FILES}
  DESTINATION
    share/${PROJECT_NAME}
)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

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
