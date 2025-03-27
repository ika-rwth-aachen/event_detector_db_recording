cmake_minimum_required(VERSION 3.8)
project(event_detector_db_recording_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(actionlib_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_cam_ts_msgs REQUIRED)
find_package(etsi_its_cpm_ts_msgs REQUIRED)
find_package(etsi_its_denm_msgs REQUIRED)
find_package(etsi_its_mapem_ts_msgs REQUIRED)
find_package(etsi_its_spatem_ts_msgs REQUIRED)
find_package(etsi_its_vam_ts_msgs REQUIRED)
find_package(event_detector REQUIRED)
find_package(event_detector_db_recording_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/mongo-cxx-driver")
find_package(mongocxx REQUIRED)

set(TARGET_NAME event_detector_db_recording_plugin)

add_library(${TARGET_NAME} SHARED
  src/DatabaseInterface.cpp
  src/RecordingRule.cpp
  src/rules/record_all/RecordAllRule.cpp
  src/rules/template/TemplateRule.cpp
  src/toBson.cpp
)

pluginlib_export_plugin_description_file(event_detector plugins.xml)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  actionlib_msgs
  builtin_interfaces
  cv_bridge
  diagnostic_msgs
  etsi_its_cam_msgs
  etsi_its_cam_ts_msgs
  etsi_its_cpm_ts_msgs
  etsi_its_denm_msgs
  etsi_its_mapem_ts_msgs
  etsi_its_spatem_ts_msgs
  etsi_its_vam_ts_msgs
  event_detector
  event_detector_db_recording_msgs
  geometry_msgs
  lifecycle_msgs
  mongocxx
  nav_msgs
  pcl_conversions
  pcl_ros
  perception_msgs
  pluginlib
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
  stereo_msgs
  tf2_msgs
  trajectory_msgs
  vision_msgs
  visualization_msgs
)

target_link_libraries(${TARGET_NAME}
  mongo::mongocxx_shared
)

install(TARGETS ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
