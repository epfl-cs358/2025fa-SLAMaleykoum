# cmake_minimum_required(VERSION 2.8.3)
# project(loam_back_and_forth)

# find_package(catkin REQUIRED COMPONENTS
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   roscpp
#   rospy
#   std_msgs
#   pcl_conversions
#   pcl_ros
#   tf)

# find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# include_directories(
#   #include
#         ${catkin_INCLUDE_DIRS}
#         ${EIGEN3_INCLUDE_DIR}
#         ${PCL_INCLUDE_DIRS})

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#   DEPENDS EIGEN3 PCL OpenCV
#   #INCLUDE_DIRS include
# )

# add_executable(scanRegistration_bf src/scanRegistration.cpp)
# target_link_libraries(scanRegistration_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(laserOdometry_bf src/laserOdometry.cpp)
# target_link_libraries(laserOdometry_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(laserMapping_bf src/laserMapping.cpp)
# target_link_libraries(laserMapping_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(transformMaintenance_bf src/transformMaintenance.cpp)
# target_link_libraries(transformMaintenance_bf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


# #if (CATKIN_ENABLE_TESTING)
# #  find_package(rostest REQUIRED)
# #  # TODO: Download test data
# #  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
# #    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
# #    MD5 3d5194e6981975588b7a93caebf79ba4)
# #  add_custom_target(${PROJECT_NAME}_test_data
# #    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
# #    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
# #  configure_file(tests/loam.test.in
# #                 ${PROJECT_BINARY_DIR}/test/loam.test)
# #  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
# #    DEPENDENCIES
# #      ${PROJECT_NAME}_test_data
# #      scanRegistration
# #      laserOdometry
# #      laserMapping
# #      transformMaintenance)
# #endif()


cmake_minimum_required(VERSION 3.14.4)
project(loam_back_and_forth)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  src
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# List of nodes to build
add_executable(scanRegistration_bf src/scanRegistration.cpp)
add_executable(laserOdometry_bf     src/laserOdometry.cpp)
add_executable(laserMapping_bf      src/laserMapping.cpp)
add_executable(transformMaintenance_bf src/transformMaintenance.cpp)

# Link them with required libraries
target_link_libraries(scanRegistration_bf ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(laserOdometry_bf     ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(laserMapping_bf      ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(transformMaintenance_bf ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Let ROS 2 know the dependencies for each target
ament_target_dependencies(scanRegistration_bf
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)

ament_target_dependencies(laserOdometry_bf
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)

ament_target_dependencies(laserMapping_bf
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)

ament_target_dependencies(transformMaintenance_bf
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install rviz config
install(
  DIRECTORY rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  scanRegistration_bf
  laserOdometry_bf
  laserMapping_bf
  transformMaintenance_bf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()