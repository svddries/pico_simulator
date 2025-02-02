cmake_minimum_required(VERSION 3.5)
project(emc_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(shape_msgs REQUIRED)

set(GEOLIB2_INCLUDE_DIR "/home/wiktor/Desktop/mrc/geolib2/install/geolib2/include")
set(GEOLIB2_LIBRARY "/home/wiktor/Desktop/mrc/geolib2/install/geolib2/lib/libgeolib.a")

# Include directories
include_directories(
  include
  SYSTEM
  3rdparty
  ${GEOLIB2_INCLUDE_DIR}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${shape_msgs_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/3rdparty/polypartition # Add this line
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

# Link libraries
target_link_libraries(pico_simulator
  polypartition
  ${rclcpp_LIBRARIES}
  ${GEOLIB2_LIBRARY} 
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${OpenCV_LIBS}
  ${image_geometry_LIBRARIES}
  ${shape_msgs_LIBRARIES}
)

ament_target_dependencies(pico_simulator tf2 tf2_ros rclcpp nav_msgs visualization_msgs geometry_msgs OpenCV image_geometry shape_msgs)


# Install targets
install(TARGETS pico_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(rclcpp tf2 nav_msgs visualization_msgs geometry_msgs OpenCV image_geometry shape_msgs)
ament_package()
