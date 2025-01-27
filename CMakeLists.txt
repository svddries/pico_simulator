cmake_minimum_required(VERSION 3.5)
project(emc_simulator)

set(CMAKE_PREFIX_PATH "/home/wiktor/Desktop/mrc/geolib2/install/geolib2/share/geolib2/cmake" ${CMAKE_PREFIX_PATH})
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geolib2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  SYSTEM
  3rdparty
  ${rclcpp_INCLUDE_DIRS}
  ${geolib2_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_library(polypartition
    3rdparty/polypartition/polypartition.cpp
)

add_executable(pico_simulator
  src/main.cpp
  src/world.cpp
  src/heightmap.cpp
  src/visualization.cpp
  src/virtualbase.cpp
  src/lrf.cpp
  src/robot.cpp
  src/moving_object.cpp
  src/bumper.cpp
)


target_link_libraries(pico_simulator
  polypartition
  ${rclcpp_LIBRARIES}
  ${geolib2_LIBRARIES}
  ${tf2_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

ament_target_dependencies(pico_simulator geolib2 tf2 rclcpp nav_msgs visualization_msgs geometry_msgs)

# Install targets
install(TARGETS pico_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(rclcpp geolib2 tf2 nav_msgs visualization_msgs geometry_msgs)
ament_package()
