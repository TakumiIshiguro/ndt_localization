cmake_minimum_required(VERSION 3.16)
project(ndt_localization LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# Ref: autoware_ndt_scan_matcher/CMakeLists.txt. Everything linked to ndt_omp must use the same
# SIMD set, otherwise Eigen's aligned members crash at startup.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
set(warnings -Wall -Wextra -Wpedantic)

# autoware_ndt_scan_matcher: multigrid_ndt_omp (vendored, BSD-2 / Apache-2.0, see ndt_omp/LICENSE)
add_library(multigrid_ndt_omp SHARED
  src/ndt_scan_matcher/ndt_omp/multi_voxel_grid_covariance_omp.cpp
  src/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.cpp)
target_include_directories(multigrid_ndt_omp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(multigrid_ndt_omp PUBLIC ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

# autoware_localization_util
add_library(localization_util SHARED
  src/localization_util/util_func.cpp
  src/localization_util/smart_pose_buffer.cpp)
target_include_directories(localization_util PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(localization_util PUBLIC Eigen3::Eigen)
ament_target_dependencies(localization_util PUBLIC rclcpp geometry_msgs tf2 tf2_eigen tf2_geometry_msgs)
target_compile_options(localization_util PRIVATE ${warnings})

# autoware_ndt_scan_matcher (+ in-process map loading and sensor preprocessing)
add_library(ndt_scan_matcher SHARED
  src/ndt_scan_matcher/ndt_scan_matcher_core.cpp
  src/ndt_scan_matcher/ndt_scan_matcher_helper.cpp
  src/map_loader/pointcloud_map_loader.cpp
  src/pointcloud_preprocessor/pointcloud_preprocessor.cpp)
target_include_directories(ndt_scan_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(ndt_scan_matcher localization_util multigrid_ndt_omp)
ament_target_dependencies(ndt_scan_matcher rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs
  diagnostic_msgs tf2_ros pcl_conversions)
target_compile_options(ndt_scan_matcher PRIVATE ${warnings})
# autoware registers with ROS2_EXECUTOR MultiThreadedExecutor (autoware_agnocast_wrapper_register_node)
rclcpp_components_register_node(ndt_scan_matcher
  PLUGIN "ndt_localization::ndt_scan_matcher::NDTScanMatcher"
  EXECUTABLE ndt_scan_matcher_node
  EXECUTOR MultiThreadedExecutor)

install(TARGETS multigrid_ndt_omp localization_util ndt_scan_matcher
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Test file names follow autoware_localization_util/test and autoware_ndt_scan_matcher/test.
  foreach(test_name util_func smart_pose_buffer)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} localization_util)
  endforeach()
  foreach(test_name ndt_scan_matcher_helper pointcloud_preprocessor)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} ndt_scan_matcher)
  endforeach()
  ament_add_gtest(test_ndt_omp test/test_ndt_omp.cpp TIMEOUT 120)
  target_link_libraries(test_ndt_omp multigrid_ndt_omp)
endif()
ament_package()
