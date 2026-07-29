cmake_minimum_required(VERSION 3.10)
project(cartesian_command_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)

generate_parameter_library(
  cartesian_command_manager_parameters
  src/cartesian_command_manager_parameters.yaml
)

add_library(${PROJECT_NAME}_core
  src/core/command_pipeline.cpp
  src/core/input_manager.cpp
  src/core/shapers/behaviour/joint_target.cpp
  src/core/shapers/geometric/jaco.cpp
  src/core/shapers/geometric/snake.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_core
  Eigen3
)

add_library(${PROJECT_NAME}_ros
  src/ros/cartesian_command_manager.cpp
  src/ros/parameter_parsing.cpp
)
target_compile_features(${PROJECT_NAME}_ros PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}_core
  cartesian_command_manager_parameters
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_executable(cartesian_command_manager_node
  src/ros/main.cpp
)
target_compile_features(cartesian_command_manager_node PUBLIC cxx_std_17)
target_link_libraries(cartesian_command_manager_node
  ${PROJECT_NAME}_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY bringup/config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_ros
    cartesian_command_manager_parameters
    cartesian_command_manager_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_ros)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  generate_parameter_library
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)


ament_package()
