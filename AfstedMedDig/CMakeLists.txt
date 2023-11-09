cmake_minimum_required(VERSION 3.8)
project(rb3_package_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rb3_node_cpp src/rb3_node_cpp.cpp)
ament_target_dependencies(rb3_node_cpp rclcpp geometry_msgs)

# Link the FFTW3 and PortAudio libraries to your executable
target_link_libraries(rb3_node_cpp portaudio)
target_link_libraries(rb3_node_cpp fftw3)

install(TARGETS
  rb3_node_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()

