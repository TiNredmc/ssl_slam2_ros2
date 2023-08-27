cmake_minimum_required(VERSION 3.5)
project(ssl_slam2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer thread)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

include_directories(include)
ament_export_include_directories(include)

#catkin_package(
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS EIGEN3 PCL  
#  INCLUDE_DIRS include
#)

set(SSL_SLAM2_DEPS 
  rclcpp 
  rclcpp_components 
  rclpy
  tf2
  tf2_ros 
  tf2_geometry_msgs 
  tf2_sensor_msgs 
  tf2_eigen 
  eigen3_cmake_module
  geometry_msgs 
  sensor_msgs
  nav_msgs
  std_srvs
  std_msgs
  pcl_msgs
  pcl_conversions
)

add_executable(ssl_slam2_laser_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
target_link_libraries(
ssl_slam2_laser_processing_node 
#${SSL_SLAM2_DEPS}
${EIGEN3_LIBRARIES} 
${PCL_LIBRARIES} 
${CERES_LIBRARIES})

ament_target_dependencies(
	ssl_slam2_laser_processing_node
	${SSL_SLAM2_DEPS}
)

add_executable(ssl_slam2_odom_estimation_localization_node src/odomEstimationLocalizationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationLocalizationClass.cpp)
target_link_libraries(
ssl_slam2_odom_estimation_localization_node 
#${SSL_SLAM2_DEPS}
${EIGEN3_LIBRARIES} 
${PCL_LIBRARIES} 
${CERES_LIBRARIES})

ament_target_dependencies(
    ssl_slam2_odom_estimation_localization_node
	${SSL_SLAM2_DEPS}
)

add_executable(ssl_slam2_odom_estimation_mapping_node src/odomEstimationMappingNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationMappingClass.cpp)
target_link_libraries(
ssl_slam2_odom_estimation_mapping_node 
#${SSL_SLAM2_DEPS}
${EIGEN3_LIBRARIES} 
${PCL_LIBRARIES} 
${CERES_LIBRARIES})

ament_target_dependencies(
	ssl_slam2_odom_estimation_mapping_node
	${SSL_SLAM2_DEPS}
)

add_executable(ssl_slam2_map_optimization_node src/mapOptimizationNode.cpp src/mapOptimizationClass.cpp src/lidar.cpp src/lidarOptimization.cpp)
target_link_libraries(
ssl_slam2_map_optimization_node 
#${SSL_SLAM2_DEPS}
${EIGEN3_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES} 
gtsam)

ament_target_dependencies(
	ssl_slam2_map_optimization_node 
	${SSL_SLAM2_DEPS}
)

install(TARGETS
	ssl_slam2_laser_processing_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    ssl_slam2_odom_estimation_localization_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	ssl_slam2_odom_estimation_mapping_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	ssl_slam2_map_optimization_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 launch
 rviz
 DESTINATION share/${PROJECT_NAME}
)

ament_package()
