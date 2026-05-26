cmake_minimum_required(VERSION 3.10)
project(scan_context_slam)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Headers
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -------------------------
# ScanContext library
# -------------------------
add_library(scancontext
    src/Scancontext.cpp
)
target_include_directories(scancontext PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scancontext PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

# -------------------------
# Module 1 standalone test
# -------------------------
add_executable(sc_test test/sc_test.cpp)
target_include_directories(sc_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sc_test
    scancontext
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

# -------------------------
# Module 2 utilities test
# -------------------------
add_executable(utilities_test  test/utilities_test.cpp)
target_include_directories(utilities_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(utilities_test
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)
target_link_libraries(utilities_test
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  gtsam
)

# -------------------------
# ROS2 Runtime
# -------------------------
install(TARGETS
  scancontext
  sc_test
  utilities_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()