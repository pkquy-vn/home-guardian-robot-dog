cmake_minimum_required(VERSION 3.8)
project(hg_perception LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# Melexis MLX90640 API (vendored unchanged, Apache-2.0) plus our Linux I2C back end.
add_library(mlx90640 STATIC
  third_party/mlx90640-library/functions/MLX90640_API.c
  src/mlx90640_linux_i2c.cpp
)
target_include_directories(mlx90640 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/headers
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mlx90640 PUBLIC m)

add_executable(thermal_node src/thermal_node.cpp)
target_compile_options(thermal_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(thermal_node
  mlx90640
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  Threads::Threads
)

install(TARGETS thermal_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Python: the reusable hg_perception module and the thermal_snapshot executable.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/thermal_snapshot DESTINATION lib/${PROJECT_NAME})

ament_package()
