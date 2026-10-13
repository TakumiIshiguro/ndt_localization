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
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
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
  src/localization_util/smart_pose_buffer.cpp
  src/localization_util/covariance_ellipse.cpp)
target_include_directories(localization_util PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(localization_util PUBLIC Eigen3::Eigen)
ament_target_dependencies(localization_util PUBLIC rclcpp geometry_msgs tf2 tf2_eigen tf2_geometry_msgs)
target_compile_options(localization_util PRIVATE ${warnings})

# autoware_kalman_filter (vendored unchanged except the namespace and include prefix, Apache-2.0)
add_library(kalman_filter SHARED
  src/kalman_filter/kalman_filter.cpp
  src/kalman_filter/time_delay_kalman_filter.cpp)
target_include_directories(kalman_filter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

# autoware_ekf_localizer (publishes map->odom instead of map->base_link when misc.odom_frame_id is set)
add_library(ekf_localizer SHARED
  src/ekf_localizer/ekf_localizer.cpp
  src/ekf_localizer/ekf_localizer_node.cpp
  src/ekf_localizer/utils/covariance.cpp
  src/ekf_localizer/utils/diagnostics.cpp
  src/ekf_localizer/utils/hyper_parameters.cpp
  src/ekf_localizer/utils/mahalanobis.cpp
  src/ekf_localizer/utils/measurement.cpp
  src/ekf_localizer/utils/state_transition.cpp
  src/ekf_localizer/utils/warning_message.cpp)
target_include_directories(ekf_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ekf_localizer>)
target_link_libraries(ekf_localizer kalman_filter localization_util fmt::fmt)
ament_target_dependencies(ekf_localizer rclcpp rclcpp_components geometry_msgs nav_msgs diagnostic_msgs std_srvs
  tf2 tf2_ros tf2_geometry_msgs)
target_compile_options(ekf_localizer PRIVATE ${warnings})
rclcpp_components_register_node(ekf_localizer
  PLUGIN "ndt_localization::ekf_localizer::EKFLocalizerNode"
  EXECUTABLE ekf_localizer_node)

# autoware_gyro_odometer
add_library(gyro_odometer SHARED
  src/gyro_odometer/gyro_odometer.cpp
  src/gyro_odometer/gyro_odometer_diagnostics.cpp
  src/gyro_odometer/gyro_odometer_node.cpp)
target_include_directories(gyro_odometer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/gyro_odometer>)
target_link_libraries(gyro_odometer fmt::fmt)
ament_target_dependencies(gyro_odometer rclcpp rclcpp_components geometry_msgs sensor_msgs diagnostic_msgs
  tf2 tf2_ros tf2_geometry_msgs)
target_compile_options(gyro_odometer PRIVATE ${warnings})
rclcpp_components_register_node(gyro_odometer
  PLUGIN "ndt_localization::gyro_odometer::GyroOdometerNode"
  EXECUTABLE gyro_odometer_node)

# autoware_vehicle_velocity_converter (takes the wheel nav_msgs/Odometry instead of VelocityReport)
add_library(vehicle_velocity_converter SHARED
  src/vehicle_velocity_converter/vehicle_velocity_converter.cpp
  src/vehicle_velocity_converter/vehicle_velocity_converter_node.cpp)
target_include_directories(vehicle_velocity_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle_velocity_converter>)
ament_target_dependencies(vehicle_velocity_converter rclcpp rclcpp_components geometry_msgs nav_msgs tf2_geometry_msgs)
target_compile_options(vehicle_velocity_converter PRIVATE ${warnings})
rclcpp_components_register_node(vehicle_velocity_converter
  PLUGIN "ndt_localization::vehicle_velocity_converter::VehicleVelocityConverterNode"
  EXECUTABLE vehicle_velocity_converter_node)

# autoware_pose_initializer (user-defined initial pose and RViz /initialpose only)
add_library(pose_initializer SHARED
  src/pose_initializer/pose_initializer.cpp
  src/pose_initializer/pose_initializer_node.cpp)
target_include_directories(pose_initializer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(pose_initializer rclcpp rclcpp_components geometry_msgs std_srvs)
target_compile_options(pose_initializer PRIVATE ${warnings})
rclcpp_components_register_node(pose_initializer
  PLUGIN "ndt_localization::pose_initializer::PoseInitializerNode"
  EXECUTABLE pose_initializer_node)

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

install(TARGETS multigrid_ndt_omp localization_util kalman_filter ekf_localizer
  gyro_odometer vehicle_velocity_converter pose_initializer ndt_scan_matcher
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Test file names follow autoware_localization_util/test and autoware_ndt_scan_matcher/test.
  foreach(test_name util_func smart_pose_buffer covariance_ellipse)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} localization_util)
  endforeach()
  foreach(test_name ndt_scan_matcher_helper pointcloud_preprocessor)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} ndt_scan_matcher)
  endforeach()
  ament_add_gtest(test_ndt_omp test/test_ndt_omp.cpp TIMEOUT 120)
  target_link_libraries(test_ndt_omp multigrid_ndt_omp)
  # autoware_kalman_filter/test
  foreach(test_name kalman_filter time_delay_kalman_filter)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} kalman_filter)
  endforeach()
  # autoware_ekf_localizer/test (same file layout)
  file(GLOB_RECURSE ekf_localizer_test_files test/ekf_localizer/*.cpp)
  foreach(filepath ${ekf_localizer_test_files})
    get_filename_component(test_name ${filepath} NAME_WE)
    ament_add_gtest(${test_name} ${filepath})
    target_link_libraries(${test_name} ekf_localizer)
  endforeach()
  # autoware_gyro_odometer/test
  foreach(test_name gyro_odometer gyro_odometer_diagnostics gyro_odometer_node)
    ament_add_gtest(test_${test_name} test/gyro_odometer/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} gyro_odometer)
  endforeach()
  ament_add_gtest(test_vehicle_velocity_converter
    test/vehicle_velocity_converter/test_vehicle_velocity_converter.cpp)
  target_link_libraries(test_vehicle_velocity_converter vehicle_velocity_converter)
  ament_add_gtest(test_pose_initializer test/test_pose_initializer.cpp)
  target_link_libraries(test_pose_initializer pose_initializer)
endif()
ament_package()
