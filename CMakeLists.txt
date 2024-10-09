cmake_minimum_required(VERSION 3.8)
project(gps_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
    src/gps_listener.cpp 
)

#Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components geometry_msgs)

# Install .sh script
install(PROGRAMS
  start_gps_cohda.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
