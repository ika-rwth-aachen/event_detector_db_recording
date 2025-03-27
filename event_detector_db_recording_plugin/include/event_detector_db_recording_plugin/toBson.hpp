/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <etsi_its_cam_msgs/msg/acceleration_confidence.hpp>
#include <etsi_its_cam_msgs/msg/acceleration_control.hpp>
#include <etsi_its_cam_msgs/msg/altitude.hpp>
#include <etsi_its_cam_msgs/msg/altitude_confidence.hpp>
#include <etsi_its_cam_msgs/msg/altitude_value.hpp>
#include <etsi_its_cam_msgs/msg/basic_container.hpp>
#include <etsi_its_cam_msgs/msg/basic_vehicle_container_high_frequency.hpp>
#include <etsi_its_cam_msgs/msg/basic_vehicle_container_low_frequency.hpp>
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_cam_msgs/msg/cam_parameters.hpp>
#include <etsi_its_cam_msgs/msg/cause_code.hpp>
#include <etsi_its_cam_msgs/msg/cause_code_type.hpp>
#include <etsi_its_cam_msgs/msg/cen_dsrc_tolling_zone.hpp>
#include <etsi_its_cam_msgs/msg/cen_dsrc_tolling_zone_id.hpp>
#include <etsi_its_cam_msgs/msg/closed_lanes.hpp>
#include <etsi_its_cam_msgs/msg/coop_awareness.hpp>
#include <etsi_its_cam_msgs/msg/curvature.hpp>
#include <etsi_its_cam_msgs/msg/curvature_calculation_mode.hpp>
#include <etsi_its_cam_msgs/msg/curvature_confidence.hpp>
#include <etsi_its_cam_msgs/msg/curvature_value.hpp>
#include <etsi_its_cam_msgs/msg/dangerous_goods_basic.hpp>
#include <etsi_its_cam_msgs/msg/dangerous_goods_container.hpp>
#include <etsi_its_cam_msgs/msg/delta_altitude.hpp>
#include <etsi_its_cam_msgs/msg/delta_latitude.hpp>
#include <etsi_its_cam_msgs/msg/delta_longitude.hpp>
#include <etsi_its_cam_msgs/msg/delta_reference_position.hpp>
#include <etsi_its_cam_msgs/msg/drive_direction.hpp>
#include <etsi_its_cam_msgs/msg/driving_lane_status.hpp>
#include <etsi_its_cam_msgs/msg/embarkation_status.hpp>
#include <etsi_its_cam_msgs/msg/emergency_container.hpp>
#include <etsi_its_cam_msgs/msg/emergency_priority.hpp>
#include <etsi_its_cam_msgs/msg/exterior_lights.hpp>
#include <etsi_its_cam_msgs/msg/generation_delta_time.hpp>
#include <etsi_its_cam_msgs/msg/hard_shoulder_status.hpp>
#include <etsi_its_cam_msgs/msg/heading.hpp>
#include <etsi_its_cam_msgs/msg/heading_confidence.hpp>
#include <etsi_its_cam_msgs/msg/heading_value.hpp>
#include <etsi_its_cam_msgs/msg/high_frequency_container.hpp>
#include <etsi_its_cam_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_cam_msgs/msg/lane_position.hpp>
#include <etsi_its_cam_msgs/msg/lateral_acceleration.hpp>
#include <etsi_its_cam_msgs/msg/lateral_acceleration_value.hpp>
#include <etsi_its_cam_msgs/msg/latitude.hpp>
#include <etsi_its_cam_msgs/msg/light_bar_siren_in_use.hpp>
#include <etsi_its_cam_msgs/msg/longitude.hpp>
#include <etsi_its_cam_msgs/msg/longitudinal_acceleration.hpp>
#include <etsi_its_cam_msgs/msg/longitudinal_acceleration_value.hpp>
#include <etsi_its_cam_msgs/msg/low_frequency_container.hpp>
#include <etsi_its_cam_msgs/msg/path_delta_time.hpp>
#include <etsi_its_cam_msgs/msg/path_history.hpp>
#include <etsi_its_cam_msgs/msg/path_point.hpp>
#include <etsi_its_cam_msgs/msg/performance_class.hpp>
#include <etsi_its_cam_msgs/msg/pos_confidence_ellipse.hpp>
#include <etsi_its_cam_msgs/msg/protected_communication_zone.hpp>
#include <etsi_its_cam_msgs/msg/protected_communication_zones_rsu.hpp>
#include <etsi_its_cam_msgs/msg/protected_zone_id.hpp>
#include <etsi_its_cam_msgs/msg/protected_zone_radius.hpp>
#include <etsi_its_cam_msgs/msg/protected_zone_type.hpp>
#include <etsi_its_cam_msgs/msg/pt_activation.hpp>
#include <etsi_its_cam_msgs/msg/pt_activation_data.hpp>
#include <etsi_its_cam_msgs/msg/pt_activation_type.hpp>
#include <etsi_its_cam_msgs/msg/public_transport_container.hpp>
#include <etsi_its_cam_msgs/msg/rsu_container_high_frequency.hpp>
#include <etsi_its_cam_msgs/msg/reference_position.hpp>
#include <etsi_its_cam_msgs/msg/rescue_container.hpp>
#include <etsi_its_cam_msgs/msg/road_works_container_basic.hpp>
#include <etsi_its_cam_msgs/msg/roadworks_sub_cause_code.hpp>
#include <etsi_its_cam_msgs/msg/safety_car_container.hpp>
#include <etsi_its_cam_msgs/msg/semi_axis_length.hpp>
#include <etsi_its_cam_msgs/msg/special_transport_container.hpp>
#include <etsi_its_cam_msgs/msg/special_transport_type.hpp>
#include <etsi_its_cam_msgs/msg/special_vehicle_container.hpp>
#include <etsi_its_cam_msgs/msg/speed.hpp>
#include <etsi_its_cam_msgs/msg/speed_confidence.hpp>
#include <etsi_its_cam_msgs/msg/speed_limit.hpp>
#include <etsi_its_cam_msgs/msg/speed_value.hpp>
#include <etsi_its_cam_msgs/msg/station_id.hpp>
#include <etsi_its_cam_msgs/msg/station_type.hpp>
#include <etsi_its_cam_msgs/msg/steering_wheel_angle.hpp>
#include <etsi_its_cam_msgs/msg/steering_wheel_angle_confidence.hpp>
#include <etsi_its_cam_msgs/msg/steering_wheel_angle_value.hpp>
#include <etsi_its_cam_msgs/msg/sub_cause_code_type.hpp>
#include <etsi_its_cam_msgs/msg/timestamp_its.hpp>
#include <etsi_its_cam_msgs/msg/traffic_rule.hpp>
#include <etsi_its_cam_msgs/msg/vehicle_length.hpp>
#include <etsi_its_cam_msgs/msg/vehicle_length_confidence_indication.hpp>
#include <etsi_its_cam_msgs/msg/vehicle_length_value.hpp>
#include <etsi_its_cam_msgs/msg/vehicle_role.hpp>
#include <etsi_its_cam_msgs/msg/vehicle_width.hpp>
#include <etsi_its_cam_msgs/msg/vertical_acceleration.hpp>
#include <etsi_its_cam_msgs/msg/vertical_acceleration_value.hpp>
#include <etsi_its_cam_msgs/msg/yaw_rate.hpp>
#include <etsi_its_cam_msgs/msg/yaw_rate_confidence.hpp>
#include <etsi_its_cam_msgs/msg/yaw_rate_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/acceleration_component.hpp>
#include <etsi_its_cam_ts_msgs/msg/acceleration_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/acceleration_control.hpp>
#include <etsi_its_cam_ts_msgs/msg/acceleration_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/accident_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/adverse_weather_condition_adhesion_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/adverse_weather_condition_extreme_weather_condition_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/adverse_weather_condition_precipitation_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/adverse_weather_condition_visibility_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/altitude.hpp>
#include <etsi_its_cam_ts_msgs/msg/altitude_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/altitude_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/basic_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/basic_vehicle_container_high_frequency.hpp>
#include <etsi_its_cam_ts_msgs/msg/basic_vehicle_container_low_frequency.hpp>
#include <etsi_its_cam_ts_msgs/msg/cam.hpp>
#include <etsi_its_cam_ts_msgs/msg/cam_parameters.hpp>
#include <etsi_its_cam_ts_msgs/msg/cam_payload.hpp>
#include <etsi_its_cam_ts_msgs/msg/cause_code_choice.hpp>
#include <etsi_its_cam_ts_msgs/msg/cause_code_v2.hpp>
#include <etsi_its_cam_ts_msgs/msg/cen_dsrc_tolling_zone.hpp>
#include <etsi_its_cam_ts_msgs/msg/closed_lanes.hpp>
#include <etsi_its_cam_ts_msgs/msg/collision_risk_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/curvature.hpp>
#include <etsi_its_cam_ts_msgs/msg/curvature_calculation_mode.hpp>
#include <etsi_its_cam_ts_msgs/msg/curvature_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/curvature_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/dangerous_end_of_queue_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/dangerous_goods_basic.hpp>
#include <etsi_its_cam_ts_msgs/msg/dangerous_goods_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/dangerous_situation_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/delta_altitude.hpp>
#include <etsi_its_cam_ts_msgs/msg/delta_latitude.hpp>
#include <etsi_its_cam_ts_msgs/msg/delta_longitude.hpp>
#include <etsi_its_cam_ts_msgs/msg/delta_reference_position.hpp>
#include <etsi_its_cam_ts_msgs/msg/drive_direction.hpp>
#include <etsi_its_cam_ts_msgs/msg/driving_lane_status.hpp>
#include <etsi_its_cam_ts_msgs/msg/embarkation_status.hpp>
#include <etsi_its_cam_ts_msgs/msg/emergency_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/emergency_priority.hpp>
#include <etsi_its_cam_ts_msgs/msg/emergency_vehicle_approaching_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/exterior_lights.hpp>
#include <etsi_its_cam_ts_msgs/msg/generation_delta_time.hpp>
#include <etsi_its_cam_ts_msgs/msg/hard_shoulder_status.hpp>
#include <etsi_its_cam_ts_msgs/msg/hazardous_location_animal_on_the_road_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/hazardous_location_dangerous_curve_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/hazardous_location_obstacle_on_the_road_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/hazardous_location_surface_condition_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/heading.hpp>
#include <etsi_its_cam_ts_msgs/msg/heading_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/heading_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/high_frequency_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/human_presence_on_the_road_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/human_problem_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/impassability_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_cam_ts_msgs/msg/lane_position.hpp>
#include <etsi_its_cam_ts_msgs/msg/latitude.hpp>
#include <etsi_its_cam_ts_msgs/msg/light_bar_siren_in_use.hpp>
#include <etsi_its_cam_ts_msgs/msg/longitude.hpp>
#include <etsi_its_cam_ts_msgs/msg/low_frequency_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/message_id.hpp>
#include <etsi_its_cam_ts_msgs/msg/ordinal_number1_b.hpp>
#include <etsi_its_cam_ts_msgs/msg/path.hpp>
#include <etsi_its_cam_ts_msgs/msg/path_delta_time.hpp>
#include <etsi_its_cam_ts_msgs/msg/path_point.hpp>
#include <etsi_its_cam_ts_msgs/msg/performance_class.hpp>
#include <etsi_its_cam_ts_msgs/msg/position_confidence_ellipse.hpp>
#include <etsi_its_cam_ts_msgs/msg/post_crash_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/protected_communication_zone.hpp>
#include <etsi_its_cam_ts_msgs/msg/protected_communication_zones_rsu.hpp>
#include <etsi_its_cam_ts_msgs/msg/protected_zone_id.hpp>
#include <etsi_its_cam_ts_msgs/msg/protected_zone_radius.hpp>
#include <etsi_its_cam_ts_msgs/msg/protected_zone_type.hpp>
#include <etsi_its_cam_ts_msgs/msg/pt_activation.hpp>
#include <etsi_its_cam_ts_msgs/msg/pt_activation_data.hpp>
#include <etsi_its_cam_ts_msgs/msg/pt_activation_type.hpp>
#include <etsi_its_cam_ts_msgs/msg/public_transport_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/rsu_container_high_frequency.hpp>
#include <etsi_its_cam_ts_msgs/msg/railway_level_crossing_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/reference_position_with_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/rescue_and_recovery_work_in_progress_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/rescue_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/road_works_container_basic.hpp>
#include <etsi_its_cam_ts_msgs/msg/roadworks_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/safety_car_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/semi_axis_length.hpp>
#include <etsi_its_cam_ts_msgs/msg/signal_violation_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/slow_vehicle_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/special_transport_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/special_transport_type.hpp>
#include <etsi_its_cam_ts_msgs/msg/special_vehicle_container.hpp>
#include <etsi_its_cam_ts_msgs/msg/speed.hpp>
#include <etsi_its_cam_ts_msgs/msg/speed_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/speed_limit.hpp>
#include <etsi_its_cam_ts_msgs/msg/speed_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/station_id.hpp>
#include <etsi_its_cam_ts_msgs/msg/stationary_vehicle_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/steering_wheel_angle.hpp>
#include <etsi_its_cam_ts_msgs/msg/steering_wheel_angle_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/steering_wheel_angle_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/sub_cause_code_type.hpp>
#include <etsi_its_cam_ts_msgs/msg/timestamp_its.hpp>
#include <etsi_its_cam_ts_msgs/msg/traffic_condition_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/traffic_participant_type.hpp>
#include <etsi_its_cam_ts_msgs/msg/traffic_rule.hpp>
#include <etsi_its_cam_ts_msgs/msg/vehicle_breakdown_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/vehicle_length.hpp>
#include <etsi_its_cam_ts_msgs/msg/vehicle_length_confidence_indication.hpp>
#include <etsi_its_cam_ts_msgs/msg/vehicle_length_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/vehicle_role.hpp>
#include <etsi_its_cam_ts_msgs/msg/vehicle_width.hpp>
#include <etsi_its_cam_ts_msgs/msg/wgs84_angle_value.hpp>
#include <etsi_its_cam_ts_msgs/msg/wrong_way_driving_sub_cause_code.hpp>
#include <etsi_its_cam_ts_msgs/msg/yaw_rate.hpp>
#include <etsi_its_cam_ts_msgs/msg/yaw_rate_confidence.hpp>
#include <etsi_its_cam_ts_msgs/msg/yaw_rate_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration3d_with_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_cartesian.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_component.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_magnitude.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_magnitude_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_polar_with_z.hpp>
#include <etsi_its_cpm_ts_msgs/msg/acceleration_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/altitude.hpp>
#include <etsi_its_cpm_ts_msgs/msg/altitude_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/altitude_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/angle_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/angular_speed_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cardinal_number1_b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cardinal_number3b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_angle.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_angle_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_angular_velocity_component.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_angular_velocity_component_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_coordinate.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_coordinate_large.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_coordinate_small.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_coordinate_with_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_position3d.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cartesian_position3d_with_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/circular_shape.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <etsi_its_cpm_ts_msgs/msg/confidence_level.hpp>
#include <etsi_its_cpm_ts_msgs/msg/constraint_wrapped_cpm_containers.hpp>
#include <etsi_its_cpm_ts_msgs/msg/coordinate_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/correlation_cell_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/correlation_column.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cpm_container_id.hpp>
#include <etsi_its_cpm_ts_msgs/msg/cpm_payload.hpp>
#include <etsi_its_cpm_ts_msgs/msg/delta_time_milli_second_signed.hpp>
#include <etsi_its_cpm_ts_msgs/msg/elliptical_shape.hpp>
#include <etsi_its_cpm_ts_msgs/msg/euler_angles_with_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/heading_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/identifier1_b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/identifier2_b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/intersection_reference_id.hpp>
#include <etsi_its_cpm_ts_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_cpm_ts_msgs/msg/latitude.hpp>
#include <etsi_its_cpm_ts_msgs/msg/longitude.hpp>
#include <etsi_its_cpm_ts_msgs/msg/longitudinal_lane_position.hpp>
#include <etsi_its_cpm_ts_msgs/msg/longitudinal_lane_position_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/longitudinal_lane_position_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/lower_triangular_positive_semidefinite_matrices.hpp>
#include <etsi_its_cpm_ts_msgs/msg/lower_triangular_positive_semidefinite_matrix.hpp>
#include <etsi_its_cpm_ts_msgs/msg/lower_triangular_positive_semidefinite_matrix_columns.hpp>
#include <etsi_its_cpm_ts_msgs/msg/management_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/map_position.hpp>
#include <etsi_its_cpm_ts_msgs/msg/map_reference.hpp>
#include <etsi_its_cpm_ts_msgs/msg/matrix_included_components.hpp>
#include <etsi_its_cpm_ts_msgs/msg/message_id.hpp>
#include <etsi_its_cpm_ts_msgs/msg/message_rate_hz.hpp>
#include <etsi_its_cpm_ts_msgs/msg/message_rate_range.hpp>
#include <etsi_its_cpm_ts_msgs/msg/message_segmentation_info.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_class.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_class_description.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_class_with_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_dimension.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_dimension_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_dimension_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/object_perception_quality.hpp>
#include <etsi_its_cpm_ts_msgs/msg/ordinal_number1_b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/ordinal_number3b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/originating_rsu_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/originating_vehicle_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/other_sub_class.hpp>
#include <etsi_its_cpm_ts_msgs/msg/perceived_object.hpp>
#include <etsi_its_cpm_ts_msgs/msg/perceived_object_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/perceived_object_ids.hpp>
#include <etsi_its_cpm_ts_msgs/msg/perceived_objects.hpp>
#include <etsi_its_cpm_ts_msgs/msg/perception_region.hpp>
#include <etsi_its_cpm_ts_msgs/msg/perception_region_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/polygonal_shape.hpp>
#include <etsi_its_cpm_ts_msgs/msg/pos_confidence_ellipse.hpp>
#include <etsi_its_cpm_ts_msgs/msg/radial_shape.hpp>
#include <etsi_its_cpm_ts_msgs/msg/radial_shape_details.hpp>
#include <etsi_its_cpm_ts_msgs/msg/radial_shapes.hpp>
#include <etsi_its_cpm_ts_msgs/msg/radial_shapes_list.hpp>
#include <etsi_its_cpm_ts_msgs/msg/rectangular_shape.hpp>
#include <etsi_its_cpm_ts_msgs/msg/reference_position.hpp>
#include <etsi_its_cpm_ts_msgs/msg/road_segment_reference_id.hpp>
#include <etsi_its_cpm_ts_msgs/msg/semi_axis_length.hpp>
#include <etsi_its_cpm_ts_msgs/msg/sensor_information.hpp>
#include <etsi_its_cpm_ts_msgs/msg/sensor_information_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/sensor_type.hpp>
#include <etsi_its_cpm_ts_msgs/msg/sequence_of_cartesian_position3d.hpp>
#include <etsi_its_cpm_ts_msgs/msg/sequence_of_identifier1_b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/shape.hpp>
#include <etsi_its_cpm_ts_msgs/msg/speed.hpp>
#include <etsi_its_cpm_ts_msgs/msg/speed_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/speed_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/standard_length12b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/standard_length1_b.hpp>
#include <etsi_its_cpm_ts_msgs/msg/station_id.hpp>
#include <etsi_its_cpm_ts_msgs/msg/timestamp_its.hpp>
#include <etsi_its_cpm_ts_msgs/msg/traffic_participant_type.hpp>
#include <etsi_its_cpm_ts_msgs/msg/trailer_data.hpp>
#include <etsi_its_cpm_ts_msgs/msg/trailer_data_set.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vehicle_width.hpp>
#include <etsi_its_cpm_ts_msgs/msg/velocity3d_with_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/velocity_cartesian.hpp>
#include <etsi_its_cpm_ts_msgs/msg/velocity_component.hpp>
#include <etsi_its_cpm_ts_msgs/msg/velocity_component_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/velocity_polar_with_z.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_cluster_information.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_cluster_profiles.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_profile_and_subprofile.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_sub_profile_animal.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_sub_profile_bicyclist.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_sub_profile_motorcyclist.hpp>
#include <etsi_its_cpm_ts_msgs/msg/vru_sub_profile_pedestrian.hpp>
#include <etsi_its_cpm_ts_msgs/msg/wgs84_angle.hpp>
#include <etsi_its_cpm_ts_msgs/msg/wgs84_angle_confidence.hpp>
#include <etsi_its_cpm_ts_msgs/msg/wgs84_angle_value.hpp>
#include <etsi_its_cpm_ts_msgs/msg/wrapped_cpm_container.hpp>
#include <etsi_its_cpm_ts_msgs/msg/wrapped_cpm_containers.hpp>
#include <etsi_its_denm_msgs/msg/action_id.hpp>
#include <etsi_its_denm_msgs/msg/alacarte_container.hpp>
#include <etsi_its_denm_msgs/msg/altitude.hpp>
#include <etsi_its_denm_msgs/msg/altitude_confidence.hpp>
#include <etsi_its_denm_msgs/msg/altitude_value.hpp>
#include <etsi_its_denm_msgs/msg/cause_code.hpp>
#include <etsi_its_denm_msgs/msg/cause_code_type.hpp>
#include <etsi_its_denm_msgs/msg/closed_lanes.hpp>
#include <etsi_its_denm_msgs/msg/denm.hpp>
#include <etsi_its_denm_msgs/msg/dangerous_goods_basic.hpp>
#include <etsi_its_denm_msgs/msg/dangerous_goods_extended.hpp>
#include <etsi_its_denm_msgs/msg/decentralized_environmental_notification_message.hpp>
#include <etsi_its_denm_msgs/msg/delta_altitude.hpp>
#include <etsi_its_denm_msgs/msg/delta_latitude.hpp>
#include <etsi_its_denm_msgs/msg/delta_longitude.hpp>
#include <etsi_its_denm_msgs/msg/delta_reference_position.hpp>
#include <etsi_its_denm_msgs/msg/driving_lane_status.hpp>
#include <etsi_its_denm_msgs/msg/energy_storage_type.hpp>
#include <etsi_its_denm_msgs/msg/event_history.hpp>
#include <etsi_its_denm_msgs/msg/event_point.hpp>
#include <etsi_its_denm_msgs/msg/hard_shoulder_status.hpp>
#include <etsi_its_denm_msgs/msg/heading.hpp>
#include <etsi_its_denm_msgs/msg/heading_confidence.hpp>
#include <etsi_its_denm_msgs/msg/heading_value.hpp>
#include <etsi_its_denm_msgs/msg/height_lon_carr.hpp>
#include <etsi_its_denm_msgs/msg/impact_reduction_container.hpp>
#include <etsi_its_denm_msgs/msg/information_quality.hpp>
#include <etsi_its_denm_msgs/msg/itinerary_path.hpp>
#include <etsi_its_denm_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_denm_msgs/msg/lane_position.hpp>
#include <etsi_its_denm_msgs/msg/latitude.hpp>
#include <etsi_its_denm_msgs/msg/light_bar_siren_in_use.hpp>
#include <etsi_its_denm_msgs/msg/location_container.hpp>
#include <etsi_its_denm_msgs/msg/longitude.hpp>
#include <etsi_its_denm_msgs/msg/management_container.hpp>
#include <etsi_its_denm_msgs/msg/number_of_occupants.hpp>
#include <etsi_its_denm_msgs/msg/path_delta_time.hpp>
#include <etsi_its_denm_msgs/msg/path_history.hpp>
#include <etsi_its_denm_msgs/msg/path_point.hpp>
#include <etsi_its_denm_msgs/msg/phone_number.hpp>
#include <etsi_its_denm_msgs/msg/pos_cent_mass.hpp>
#include <etsi_its_denm_msgs/msg/pos_confidence_ellipse.hpp>
#include <etsi_its_denm_msgs/msg/pos_front_ax.hpp>
#include <etsi_its_denm_msgs/msg/pos_lon_carr.hpp>
#include <etsi_its_denm_msgs/msg/pos_pillar.hpp>
#include <etsi_its_denm_msgs/msg/position_of_occupants.hpp>
#include <etsi_its_denm_msgs/msg/position_of_pillars.hpp>
#include <etsi_its_denm_msgs/msg/positioning_solution_type.hpp>
#include <etsi_its_denm_msgs/msg/reference_denms.hpp>
#include <etsi_its_denm_msgs/msg/reference_position.hpp>
#include <etsi_its_denm_msgs/msg/relevance_distance.hpp>
#include <etsi_its_denm_msgs/msg/relevance_traffic_direction.hpp>
#include <etsi_its_denm_msgs/msg/request_response_indication.hpp>
#include <etsi_its_denm_msgs/msg/restricted_types.hpp>
#include <etsi_its_denm_msgs/msg/road_type.hpp>
#include <etsi_its_denm_msgs/msg/road_works_container_extended.hpp>
#include <etsi_its_denm_msgs/msg/semi_axis_length.hpp>
#include <etsi_its_denm_msgs/msg/sequence_number.hpp>
#include <etsi_its_denm_msgs/msg/situation_container.hpp>
#include <etsi_its_denm_msgs/msg/speed.hpp>
#include <etsi_its_denm_msgs/msg/speed_confidence.hpp>
#include <etsi_its_denm_msgs/msg/speed_limit.hpp>
#include <etsi_its_denm_msgs/msg/speed_value.hpp>
#include <etsi_its_denm_msgs/msg/station_id.hpp>
#include <etsi_its_denm_msgs/msg/station_type.hpp>
#include <etsi_its_denm_msgs/msg/stationary_since.hpp>
#include <etsi_its_denm_msgs/msg/stationary_vehicle_container.hpp>
#include <etsi_its_denm_msgs/msg/sub_cause_code_type.hpp>
#include <etsi_its_denm_msgs/msg/temperature.hpp>
#include <etsi_its_denm_msgs/msg/termination.hpp>
#include <etsi_its_denm_msgs/msg/timestamp_its.hpp>
#include <etsi_its_denm_msgs/msg/traces.hpp>
#include <etsi_its_denm_msgs/msg/traffic_rule.hpp>
#include <etsi_its_denm_msgs/msg/transmission_interval.hpp>
#include <etsi_its_denm_msgs/msg/turning_radius.hpp>
#include <etsi_its_denm_msgs/msg/vds.hpp>
#include <etsi_its_denm_msgs/msg/validity_duration.hpp>
#include <etsi_its_denm_msgs/msg/vehicle_identification.hpp>
#include <etsi_its_denm_msgs/msg/vehicle_mass.hpp>
#include <etsi_its_denm_msgs/msg/wm_inumber.hpp>
#include <etsi_its_denm_msgs/msg/wheel_base_vehicle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/allowed_maneuvers.hpp>
#include <etsi_its_mapem_ts_msgs/msg/angle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/approach_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/computed_lane.hpp>
#include <etsi_its_mapem_ts_msgs/msg/connecting_lane.hpp>
#include <etsi_its_mapem_ts_msgs/msg/connection.hpp>
#include <etsi_its_mapem_ts_msgs/msg/connects_to_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/data_parameters.hpp>
#include <etsi_its_mapem_ts_msgs/msg/delta_angle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/descriptive_name.hpp>
#include <etsi_its_mapem_ts_msgs/msg/driven_line_offset_lg.hpp>
#include <etsi_its_mapem_ts_msgs/msg/driven_line_offset_sm.hpp>
#include <etsi_its_mapem_ts_msgs/msg/elevation.hpp>
#include <etsi_its_mapem_ts_msgs/msg/generic_lane.hpp>
#include <etsi_its_mapem_ts_msgs/msg/intersection_geometry.hpp>
#include <etsi_its_mapem_ts_msgs/msg/intersection_geometry_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/intersection_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/intersection_reference_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_barrier.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_bike.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_crosswalk.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_parking.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_sidewalk.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_striping.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_tracked_vehicle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_attributes_vehicle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_connection_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_data_attribute.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_data_attribute_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_direction.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_sharing.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_type_attributes.hpp>
#include <etsi_its_mapem_ts_msgs/msg/lane_width.hpp>
#include <etsi_its_mapem_ts_msgs/msg/latitude.hpp>
#include <etsi_its_mapem_ts_msgs/msg/layer_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/layer_type.hpp>
#include <etsi_its_mapem_ts_msgs/msg/longitude.hpp>
#include <etsi_its_mapem_ts_msgs/msg/mapem.hpp>
#include <etsi_its_mapem_ts_msgs/msg/map_data.hpp>
#include <etsi_its_mapem_ts_msgs/msg/merge_diverge_node_angle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/minute_of_the_year.hpp>
#include <etsi_its_mapem_ts_msgs/msg/msg_count.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_attribute_set_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_attribute_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_attribute_xy_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_l_lm_d64b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_list_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_offset_point_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_set_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy20b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy22b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy24b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy26b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy28b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/node_xy32b.hpp>
#include <etsi_its_mapem_ts_msgs/msg/offset_b10.hpp>
#include <etsi_its_mapem_ts_msgs/msg/offset_b11.hpp>
#include <etsi_its_mapem_ts_msgs/msg/offset_b12.hpp>
#include <etsi_its_mapem_ts_msgs/msg/offset_b13.hpp>
#include <etsi_its_mapem_ts_msgs/msg/offset_b14.hpp>
#include <etsi_its_mapem_ts_msgs/msg/offset_b16.hpp>
#include <etsi_its_mapem_ts_msgs/msg/overlay_lane_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/position3_d.hpp>
#include <etsi_its_mapem_ts_msgs/msg/preempt_priority_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/regulatory_speed_limit.hpp>
#include <etsi_its_mapem_ts_msgs/msg/restriction_applies_to.hpp>
#include <etsi_its_mapem_ts_msgs/msg/restriction_class_assignment.hpp>
#include <etsi_its_mapem_ts_msgs/msg/restriction_class_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/restriction_class_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/restriction_user_type.hpp>
#include <etsi_its_mapem_ts_msgs/msg/restriction_user_type_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/road_lane_set_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/road_regulator_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/road_segment.hpp>
#include <etsi_its_mapem_ts_msgs/msg/road_segment_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/road_segment_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/road_segment_reference_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/roadway_crown_angle.hpp>
#include <etsi_its_mapem_ts_msgs/msg/scale_b12.hpp>
#include <etsi_its_mapem_ts_msgs/msg/segment_attribute_xy.hpp>
#include <etsi_its_mapem_ts_msgs/msg/segment_attribute_xy_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/signal_control_zone.hpp>
#include <etsi_its_mapem_ts_msgs/msg/signal_group_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/speed_limit_list.hpp>
#include <etsi_its_mapem_ts_msgs/msg/speed_limit_type.hpp>
#include <etsi_its_mapem_ts_msgs/msg/station_id.hpp>
#include <etsi_its_mapem_ts_msgs/msg/velocity.hpp>
#include <etsi_its_spatem_ts_msgs/msg/advisory_speed.hpp>
#include <etsi_its_spatem_ts_msgs/msg/advisory_speed_list.hpp>
#include <etsi_its_spatem_ts_msgs/msg/advisory_speed_type.hpp>
#include <etsi_its_spatem_ts_msgs/msg/connection_maneuver_assist.hpp>
#include <etsi_its_spatem_ts_msgs/msg/d_second.hpp>
#include <etsi_its_spatem_ts_msgs/msg/descriptive_name.hpp>
#include <etsi_its_spatem_ts_msgs/msg/enabled_lane_list.hpp>
#include <etsi_its_spatem_ts_msgs/msg/intersection_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/intersection_reference_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/intersection_state.hpp>
#include <etsi_its_spatem_ts_msgs/msg/intersection_state_list.hpp>
#include <etsi_its_spatem_ts_msgs/msg/intersection_status_object.hpp>
#include <etsi_its_spatem_ts_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_spatem_ts_msgs/msg/lane_connection_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/lane_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/maneuver_assist_list.hpp>
#include <etsi_its_spatem_ts_msgs/msg/minute_of_the_year.hpp>
#include <etsi_its_spatem_ts_msgs/msg/movement_event.hpp>
#include <etsi_its_spatem_ts_msgs/msg/movement_event_list.hpp>
#include <etsi_its_spatem_ts_msgs/msg/movement_list.hpp>
#include <etsi_its_spatem_ts_msgs/msg/movement_phase_state.hpp>
#include <etsi_its_spatem_ts_msgs/msg/movement_state.hpp>
#include <etsi_its_spatem_ts_msgs/msg/msg_count.hpp>
#include <etsi_its_spatem_ts_msgs/msg/pedestrian_bicycle_detect.hpp>
#include <etsi_its_spatem_ts_msgs/msg/restriction_class_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/road_regulator_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/spat.hpp>
#include <etsi_its_spatem_ts_msgs/msg/spatem.hpp>
#include <etsi_its_spatem_ts_msgs/msg/signal_group_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/speed_advice.hpp>
#include <etsi_its_spatem_ts_msgs/msg/speed_confidence_dsrc.hpp>
#include <etsi_its_spatem_ts_msgs/msg/station_id.hpp>
#include <etsi_its_spatem_ts_msgs/msg/time_change_details.hpp>
#include <etsi_its_spatem_ts_msgs/msg/time_interval_confidence.hpp>
#include <etsi_its_spatem_ts_msgs/msg/time_mark.hpp>
#include <etsi_its_spatem_ts_msgs/msg/wait_on_stopline.hpp>
#include <etsi_its_spatem_ts_msgs/msg/zone_length.hpp>
#include <etsi_its_vam_ts_msgs/msg/acceleration_change.hpp>
#include <etsi_its_vam_ts_msgs/msg/acceleration_change_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/acceleration_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/altitude.hpp>
#include <etsi_its_vam_ts_msgs/msg/altitude_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/altitude_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/angle_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/basic_container.hpp>
#include <etsi_its_vam_ts_msgs/msg/cardinal_number1_b.hpp>
#include <etsi_its_vam_ts_msgs/msg/cartesian_angle.hpp>
#include <etsi_its_vam_ts_msgs/msg/cartesian_angle_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/cartesian_coordinate.hpp>
#include <etsi_its_vam_ts_msgs/msg/cartesian_coordinate_small.hpp>
#include <etsi_its_vam_ts_msgs/msg/cartesian_position3d.hpp>
#include <etsi_its_vam_ts_msgs/msg/circular_shape.hpp>
#include <etsi_its_vam_ts_msgs/msg/cluster_breakup_info.hpp>
#include <etsi_its_vam_ts_msgs/msg/cluster_breakup_reason.hpp>
#include <etsi_its_vam_ts_msgs/msg/cluster_join_info.hpp>
#include <etsi_its_vam_ts_msgs/msg/cluster_leave_info.hpp>
#include <etsi_its_vam_ts_msgs/msg/cluster_leave_reason.hpp>
#include <etsi_its_vam_ts_msgs/msg/curvature.hpp>
#include <etsi_its_vam_ts_msgs/msg/curvature_calculation_mode.hpp>
#include <etsi_its_vam_ts_msgs/msg/curvature_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/curvature_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/delta_altitude.hpp>
#include <etsi_its_vam_ts_msgs/msg/delta_latitude.hpp>
#include <etsi_its_vam_ts_msgs/msg/delta_longitude.hpp>
#include <etsi_its_vam_ts_msgs/msg/delta_reference_position.hpp>
#include <etsi_its_vam_ts_msgs/msg/delta_time_quarter_second.hpp>
#include <etsi_its_vam_ts_msgs/msg/delta_time_tenth_of_second.hpp>
#include <etsi_its_vam_ts_msgs/msg/elliptical_shape.hpp>
#include <etsi_its_vam_ts_msgs/msg/exterior_lights.hpp>
#include <etsi_its_vam_ts_msgs/msg/generalized_lane_position.hpp>
#include <etsi_its_vam_ts_msgs/msg/generation_delta_time.hpp>
#include <etsi_its_vam_ts_msgs/msg/heading_change_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/heading_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/identifier1_b.hpp>
#include <etsi_its_vam_ts_msgs/msg/identifier2_b.hpp>
#include <etsi_its_vam_ts_msgs/msg/intersection_reference_id.hpp>
#include <etsi_its_vam_ts_msgs/msg/its_pdu_header.hpp>
#include <etsi_its_vam_ts_msgs/msg/its_pdu_header_vam.hpp>
#include <etsi_its_vam_ts_msgs/msg/lane_position.hpp>
#include <etsi_its_vam_ts_msgs/msg/lane_position_and_type.hpp>
#include <etsi_its_vam_ts_msgs/msg/lane_type.hpp>
#include <etsi_its_vam_ts_msgs/msg/lateral_acceleration.hpp>
#include <etsi_its_vam_ts_msgs/msg/lateral_acceleration_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/latitude.hpp>
#include <etsi_its_vam_ts_msgs/msg/longitude.hpp>
#include <etsi_its_vam_ts_msgs/msg/longitudinal_acceleration.hpp>
#include <etsi_its_vam_ts_msgs/msg/longitudinal_acceleration_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/longitudinal_lane_position.hpp>
#include <etsi_its_vam_ts_msgs/msg/longitudinal_lane_position_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/longitudinal_lane_position_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/map_position.hpp>
#include <etsi_its_vam_ts_msgs/msg/map_reference.hpp>
#include <etsi_its_vam_ts_msgs/msg/message_id.hpp>
#include <etsi_its_vam_ts_msgs/msg/ordinal_number1_b.hpp>
#include <etsi_its_vam_ts_msgs/msg/path_delta_time.hpp>
#include <etsi_its_vam_ts_msgs/msg/path_history.hpp>
#include <etsi_its_vam_ts_msgs/msg/path_point.hpp>
#include <etsi_its_vam_ts_msgs/msg/path_point_predicted.hpp>
#include <etsi_its_vam_ts_msgs/msg/path_predicted.hpp>
#include <etsi_its_vam_ts_msgs/msg/polygonal_shape.hpp>
#include <etsi_its_vam_ts_msgs/msg/pos_confidence_ellipse.hpp>
#include <etsi_its_vam_ts_msgs/msg/position_confidence_ellipse.hpp>
#include <etsi_its_vam_ts_msgs/msg/radial_shape.hpp>
#include <etsi_its_vam_ts_msgs/msg/radial_shape_details.hpp>
#include <etsi_its_vam_ts_msgs/msg/radial_shapes.hpp>
#include <etsi_its_vam_ts_msgs/msg/radial_shapes_list.hpp>
#include <etsi_its_vam_ts_msgs/msg/rectangular_shape.hpp>
#include <etsi_its_vam_ts_msgs/msg/reference_position_with_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/road_segment_reference_id.hpp>
#include <etsi_its_vam_ts_msgs/msg/safe_distance_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/safe_distance_indicator.hpp>
#include <etsi_its_vam_ts_msgs/msg/semi_axis_length.hpp>
#include <etsi_its_vam_ts_msgs/msg/sequence_of_cartesian_position3d.hpp>
#include <etsi_its_vam_ts_msgs/msg/sequence_of_safe_distance_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/sequence_of_trajectory_interception_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/shape.hpp>
#include <etsi_its_vam_ts_msgs/msg/speed.hpp>
#include <etsi_its_vam_ts_msgs/msg/speed_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/speed_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/stability_change_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/stability_loss_probability.hpp>
#include <etsi_its_vam_ts_msgs/msg/standard_length12b.hpp>
#include <etsi_its_vam_ts_msgs/msg/station_id.hpp>
#include <etsi_its_vam_ts_msgs/msg/traffic_island_position.hpp>
#include <etsi_its_vam_ts_msgs/msg/traffic_participant_type.hpp>
#include <etsi_its_vam_ts_msgs/msg/trajectory_interception_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/trajectory_interception_indication.hpp>
#include <etsi_its_vam_ts_msgs/msg/trajectory_interception_probability.hpp>
#include <etsi_its_vam_ts_msgs/msg/turning_direction.hpp>
#include <etsi_its_vam_ts_msgs/msg/vam.hpp>
#include <etsi_its_vam_ts_msgs/msg/vam_parameters.hpp>
#include <etsi_its_vam_ts_msgs/msg/vertical_acceleration.hpp>
#include <etsi_its_vam_ts_msgs/msg/vertical_acceleration_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_awareness.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_cluster_information.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_cluster_information_container.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_cluster_operation_container.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_cluster_profiles.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_device_usage.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_environment.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_exterior_lights.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_high_frequency_container.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_low_frequency_container.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_motion_prediction_container.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_movement_control.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_profile_and_subprofile.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_size_class.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_specific_exterior_lights.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_sub_profile_animal.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_sub_profile_bicyclist.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_sub_profile_motorcyclist.hpp>
#include <etsi_its_vam_ts_msgs/msg/vru_sub_profile_pedestrian.hpp>
#include <etsi_its_vam_ts_msgs/msg/wgs84_angle.hpp>
#include <etsi_its_vam_ts_msgs/msg/wgs84_angle_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/wgs84_angle_value.hpp>
#include <etsi_its_vam_ts_msgs/msg/yaw_rate.hpp>
#include <etsi_its_vam_ts_msgs/msg/yaw_rate_confidence.hpp>
#include <etsi_its_vam_ts_msgs/msg/yaw_rate_value.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/velocity_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object.hpp>
#include <perception_msgs/msg/object_classification.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs/msg/object_reference_point.hpp>
#include <perception_msgs/msg/object_state.hpp>
#include <perception_msgs/msg/object_state_prediction.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <tf2_msgs/msg/tf2_error.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>
#include <vision_msgs/msg/classification.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/label_info.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/point2_d.hpp>
#include <vision_msgs/msg/pose2_d.hpp>
#include <vision_msgs/msg/vision_class.hpp>
#include <vision_msgs/msg/vision_info.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_pose.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <visualization_msgs/msg/mesh_file.hpp>
#include <visualization_msgs/msg/uv_coordinate.hpp>
#include <event_detector_db_recording_msgs/msg/image_file.hpp>
#include <event_detector_db_recording_msgs/msg/point_cloud_file.hpp>


namespace event_detector_db_recording_plugin {

bsoncxx::types::b_double toBson(const float d);
bsoncxx::types::b_double toBson(const double f);
bsoncxx::types::b_string toBson(const std::string& s);
bsoncxx::types::b_bool toBson(const bool b);
bsoncxx::types::b_date toBson(const rclcpp::Time& time);
bsoncxx::types::b_double toBson(const rclcpp::Duration& duration);
bsoncxx::types::b_int32 toBson(const signed char i);
bsoncxx::types::b_int32 toBson(const unsigned char i);
bsoncxx::types::b_int32 toBson(const short int i);
bsoncxx::types::b_int32 toBson(const unsigned short int i);
bsoncxx::types::b_int32 toBson(const int i);
bsoncxx::types::b_int32 toBson(const unsigned int i);
bsoncxx::types::b_int32 toBson(const long int i);
bsoncxx::types::b_int32 toBson(const unsigned long int i);
bsoncxx::types::b_int64 toBson(const long long i);
bsoncxx::types::b_int64 toBson(const unsigned long long int i);
bsoncxx::types::b_oid toBson(const bsoncxx::oid& oid);
bsoncxx::document::value toBson(const nlohmann::json& j);

bsoncxx::document::value toBson(const builtin_interfaces::msg::Duration& msg);
bsoncxx::document::value toBson(const builtin_interfaces::msg::Time& msg);
bsoncxx::document::value toBson(const diagnostic_msgs::msg::DiagnosticArray& msg);
bsoncxx::document::value toBson(const diagnostic_msgs::msg::DiagnosticStatus& msg);
bsoncxx::document::value toBson(const diagnostic_msgs::msg::KeyValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::AccelerationConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::AccelerationControl& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::Altitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::AltitudeConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::AltitudeValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::BasicContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::BasicVehicleContainerHighFrequency& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::BasicVehicleContainerLowFrequency& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CAM& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CamParameters& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CauseCodeType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CenDsrcTollingZone& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CenDsrcTollingZoneID& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ClosedLanes& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CoopAwareness& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::Curvature& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CurvatureCalculationMode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CurvatureConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::CurvatureValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DangerousGoodsBasic& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DangerousGoodsContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DeltaAltitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DeltaLatitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DeltaLongitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DeltaReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DriveDirection& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::DrivingLaneStatus& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::EmbarkationStatus& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::EmergencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::EmergencyPriority& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ExteriorLights& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::GenerationDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::HardShoulderStatus& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::Heading& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::HeadingConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::HeadingValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::HighFrequencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LateralAcceleration& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LateralAccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::Latitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LightBarSirenInUse& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::Longitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LongitudinalAcceleration& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LongitudinalAccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::LowFrequencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PathDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PathHistory& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PathPoint& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PerformanceClass& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PosConfidenceEllipse& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ProtectedCommunicationZone& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ProtectedCommunicationZonesRSU& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ProtectedZoneID& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ProtectedZoneRadius& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ProtectedZoneType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PtActivation& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PtActivationData& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PtActivationType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::PublicTransportContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::RSUContainerHighFrequency& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::ReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::RescueContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::RoadWorksContainerBasic& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::RoadworksSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SafetyCarContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SemiAxisLength& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SpecialTransportContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SpecialTransportType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SpecialVehicleContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::Speed& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SpeedConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SpeedLimit& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SpeedValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::StationID& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::StationType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SteeringWheelAngle& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SteeringWheelAngleConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SteeringWheelAngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::SubCauseCodeType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::TimestampIts& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::TrafficRule& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VehicleLength& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VehicleLengthValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VehicleRole& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VehicleWidth& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VerticalAcceleration& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::VerticalAccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::YawRate& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::YawRateConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_msgs::msg::YawRateValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationComponent& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationControl& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AccidentSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionAdhesionSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionExtremeWeatherConditionSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionPrecipitationSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionVisibilitySubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Altitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AltitudeConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::AltitudeValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::BasicContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::BasicVehicleContainerHighFrequency& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::BasicVehicleContainerLowFrequency& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CAM& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CamParameters& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CamPayload& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CauseCodeChoice& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CauseCodeV2& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CenDsrcTollingZone& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ClosedLanes& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CollisionRiskSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Curvature& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CurvatureCalculationMode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CurvatureConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::CurvatureValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DangerousEndOfQueueSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DangerousGoodsBasic& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DangerousGoodsContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DangerousSituationSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DeltaAltitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DeltaLatitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DeltaLongitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DeltaReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DriveDirection& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::DrivingLaneStatus& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::EmbarkationStatus& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::EmergencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::EmergencyPriority& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::EmergencyVehicleApproachingSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ExteriorLights& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::GenerationDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HardShoulderStatus& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationAnimalOnTheRoadSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationDangerousCurveSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationObstacleOnTheRoadSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationSurfaceConditionSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Heading& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HeadingConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HeadingValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HighFrequencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HumanPresenceOnTheRoadSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::HumanProblemSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ImpassabilitySubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::LanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Latitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::LightBarSirenInUse& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Longitude& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::LowFrequencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::MessageId& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::OrdinalNumber1B& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Path& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PathDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PathPoint& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PerformanceClass& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PositionConfidenceEllipse& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PostCrashSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedCommunicationZone& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedCommunicationZonesRSU& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedZoneId& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedZoneRadius& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedZoneType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PtActivation& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PtActivationData& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PtActivationType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::PublicTransportContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::RSUContainerHighFrequency& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::RailwayLevelCrossingSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::ReferencePositionWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::RescueAndRecoveryWorkInProgressSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::RescueContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::RoadWorksContainerBasic& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::RoadworksSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SafetyCarContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SemiAxisLength& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SignalViolationSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SlowVehicleSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SpecialTransportContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SpecialTransportType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SpecialVehicleContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Speed& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SpeedConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SpeedLimit& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SpeedValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::StationId& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::StationaryVehicleSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SteeringWheelAngle& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SteeringWheelAngleConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SteeringWheelAngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::SubCauseCodeType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::TimestampIts& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::TrafficConditionSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::TrafficParticipantType& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::TrafficRule& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::VehicleBreakdownSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::VehicleLength& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::VehicleLengthConfidenceIndication& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::VehicleLengthValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::VehicleRole& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::VehicleWidth& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::Wgs84AngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::WrongWayDrivingSubCauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::YawRate& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::YawRateConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cam_ts_msgs::msg::YawRateValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Acceleration3dWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationCartesian& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationComponent& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationMagnitude& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationMagnitudeValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationPolarWithZ& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Altitude& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AltitudeConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AltitudeValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AngleConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::AngularSpeedConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CardinalNumber1B& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CardinalNumber3b& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngle& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngularVelocityComponent& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngularVelocityComponentValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinate& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinateLarge& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinateSmall& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinateWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianPosition3d& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianPosition3dWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CircularShape& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ConfidenceLevel& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ConstraintWrappedCpmContainers& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CoordinateConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CorrelationCellValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CorrelationColumn& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CpmContainerId& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::CpmPayload& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::DeltaTimeMilliSecondSigned& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::EllipticalShape& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::EulerAnglesWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::HeadingValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Identifier1B& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Identifier2B& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::IntersectionReferenceId& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Latitude& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Longitude& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::LongitudinalLanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::LongitudinalLanePositionConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::LongitudinalLanePositionValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::LowerTriangularPositiveSemidefiniteMatrices& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::LowerTriangularPositiveSemidefiniteMatrix& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::LowerTriangularPositiveSemidefiniteMatrixColumns& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ManagementContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MapPosition& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MapReference& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MatrixIncludedComponents& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MessageId& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MessageRateHz& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MessageRateRange& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::MessageSegmentationInfo& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectClass& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectClassDescription& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectClassWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectDimension& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectDimensionConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectDimensionValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectPerceptionQuality& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::OrdinalNumber1B& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::OrdinalNumber3b& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::OriginatingRsuContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::OriginatingVehicleContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::OtherSubClass& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObject& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObjectContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObjectIds& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObjects& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PerceptionRegion& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PerceptionRegionContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PolygonalShape& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::PosConfidenceEllipse& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShape& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShapeDetails& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShapes& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShapesList& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::RectangularShape& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::ReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::RoadSegmentReferenceId& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SemiAxisLength& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SensorInformation& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SensorInformationContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SensorType& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SequenceOfCartesianPosition3d& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SequenceOfIdentifier1B& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Shape& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Speed& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SpeedConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::SpeedValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::StandardLength12b& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::StandardLength1B& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::StationId& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::TimestampIts& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::TrafficParticipantType& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::TrailerData& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::TrailerDataSet& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VehicleWidth& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Velocity3dWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityCartesian& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityComponent& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityComponentValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityPolarWithZ& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruClusterInformation& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruClusterProfiles& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruProfileAndSubprofile& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfileAnimal& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfileBicyclist& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfileMotorcyclist& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfilePedestrian& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Wgs84Angle& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Wgs84AngleConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::Wgs84AngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::WrappedCpmContainer& msg);
bsoncxx::document::value toBson(const etsi_its_cpm_ts_msgs::msg::WrappedCpmContainers& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ActionID& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::AlacarteContainer& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Altitude& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::AltitudeConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::AltitudeValue& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::CauseCode& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::CauseCodeType& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ClosedLanes& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DENM& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DangerousGoodsBasic& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DangerousGoodsExtended& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DecentralizedEnvironmentalNotificationMessage& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DeltaAltitude& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DeltaLatitude& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DeltaLongitude& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DeltaReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::DrivingLaneStatus& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::EnergyStorageType& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::EventHistory& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::EventPoint& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::HardShoulderStatus& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Heading& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::HeadingConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::HeadingValue& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::HeightLonCarr& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ImpactReductionContainer& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::InformationQuality& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ItineraryPath& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::LanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Latitude& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::LightBarSirenInUse& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::LocationContainer& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Longitude& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ManagementContainer& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::NumberOfOccupants& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PathDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PathHistory& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PathPoint& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PhoneNumber& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PosCentMass& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PosConfidenceEllipse& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PosFrontAx& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PosLonCarr& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PosPillar& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PositionOfOccupants& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PositionOfPillars& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::PositioningSolutionType& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ReferenceDenms& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::RelevanceDistance& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::RelevanceTrafficDirection& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::RequestResponseIndication& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::RestrictedTypes& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::RoadType& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::RoadWorksContainerExtended& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SemiAxisLength& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SequenceNumber& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SituationContainer& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Speed& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SpeedConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SpeedLimit& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SpeedValue& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::StationID& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::StationType& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::StationarySince& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::StationaryVehicleContainer& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::SubCauseCodeType& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Temperature& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Termination& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::TimestampIts& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::Traces& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::TrafficRule& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::TransmissionInterval& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::TurningRadius& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::VDS& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::ValidityDuration& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::VehicleIdentification& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::VehicleMass& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::WMInumber& msg);
bsoncxx::document::value toBson(const etsi_its_denm_msgs::msg::WheelBaseVehicle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::AllowedManeuvers& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Angle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::ApproachID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::ComputedLane& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::ConnectingLane& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Connection& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::ConnectsToList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::DataParameters& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::DeltaAngle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::DescriptiveName& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::DrivenLineOffsetLg& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::DrivenLineOffsetSm& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Elevation& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::GenericLane& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionGeometry& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionGeometryList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionReferenceID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributes& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesBarrier& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesBike& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesCrosswalk& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesParking& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesSidewalk& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesStriping& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesTrackedVehicle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesVehicle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneConnectionID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneDataAttribute& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneDataAttributeList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneDirection& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneSharing& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LaneWidth& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Latitude& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LayerID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::LayerType& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Longitude& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::MAPEM& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::MapData& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::MergeDivergeNodeAngle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::MinuteOfTheYear& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::MsgCount& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeAttributeSetXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeAttributeXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeAttributeXYList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeLLmD64b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeListXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeSetXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY20b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY22b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY24b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY26b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY28b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY32b& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB10& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB11& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB12& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB13& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB14& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB16& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::OverlayLaneList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Position3D& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::PreemptPriorityList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RegulatorySpeedLimit& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionAppliesTo& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionClassAssignment& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionClassID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionClassList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionUserType& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionUserTypeList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadLaneSetList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadRegulatorID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegment& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegmentID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegmentList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegmentReferenceID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::RoadwayCrownAngle& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::ScaleB12& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::SegmentAttributeXY& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::SegmentAttributeXYList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::SignalControlZone& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::SignalGroupID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::SpeedLimitList& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::SpeedLimitType& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::StationID& msg);
bsoncxx::document::value toBson(const etsi_its_mapem_ts_msgs::msg::Velocity& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::AdvisorySpeed& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::AdvisorySpeedList& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::AdvisorySpeedType& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::ConnectionManeuverAssist& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::DSecond& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::DescriptiveName& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::EnabledLaneList& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionReferenceID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionState& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionStateList& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionStatusObject& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::LaneConnectionID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::LaneID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::ManeuverAssistList& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MinuteOfTheYear& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MovementEvent& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MovementEventList& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MovementList& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MovementPhaseState& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MovementState& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::MsgCount& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::PedestrianBicycleDetect& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::RestrictionClassID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::RoadRegulatorID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::SPAT& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::SPATEM& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::SignalGroupID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::SpeedAdvice& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::SpeedConfidenceDSRC& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::StationID& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::TimeChangeDetails& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::TimeIntervalConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::TimeMark& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::WaitOnStopline& msg);
bsoncxx::document::value toBson(const etsi_its_spatem_ts_msgs::msg::ZoneLength& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::AccelerationChange& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::AccelerationChangeIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::AccelerationConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Altitude& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::AltitudeConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::AltitudeValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::AngleConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::BasicContainer& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CardinalNumber1B& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CartesianAngle& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CartesianAngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CartesianCoordinate& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CartesianCoordinateSmall& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CartesianPosition3d& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CircularShape& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ClusterBreakupInfo& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ClusterBreakupReason& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ClusterJoinInfo& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ClusterLeaveInfo& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ClusterLeaveReason& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Curvature& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CurvatureCalculationMode& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CurvatureConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::CurvatureValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::DeltaAltitude& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::DeltaLatitude& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::DeltaLongitude& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::DeltaReferencePosition& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::DeltaTimeQuarterSecond& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::DeltaTimeTenthOfSecond& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::EllipticalShape& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ExteriorLights& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::GeneralizedLanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::GenerationDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::HeadingChangeIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::HeadingValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Identifier1B& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Identifier2B& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::IntersectionReferenceId& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ItsPduHeader& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ItsPduHeaderVam& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LanePositionAndType& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LaneType& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LateralAcceleration& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LateralAccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Latitude& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Longitude& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalAcceleration& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalAccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalLanePosition& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalLanePositionConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalLanePositionValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::MapPosition& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::MapReference& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::MessageId& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::OrdinalNumber1B& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PathDeltaTime& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PathHistory& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PathPoint& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PathPointPredicted& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PathPredicted& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PolygonalShape& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PosConfidenceEllipse& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::PositionConfidenceEllipse& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::RadialShape& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::RadialShapeDetails& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::RadialShapes& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::RadialShapesList& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::RectangularShape& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::ReferencePositionWithConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::RoadSegmentReferenceId& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SafeDistanceIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SafeDistanceIndicator& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SemiAxisLength& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SequenceOfCartesianPosition3d& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SequenceOfSafeDistanceIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SequenceOfTrajectoryInterceptionIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Shape& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Speed& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SpeedConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::SpeedValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::StabilityChangeIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::StabilityLossProbability& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::StandardLength12b& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::StationId& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::TrafficIslandPosition& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::TrafficParticipantType& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::TrajectoryInterceptionConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::TrajectoryInterceptionIndication& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::TrajectoryInterceptionProbability& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::TurningDirection& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VAM& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VamParameters& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VerticalAcceleration& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VerticalAccelerationValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruAwareness& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterInformation& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterInformationContainer& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterOperationContainer& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterProfiles& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruDeviceUsage& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruEnvironment& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruExteriorLights& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruHighFrequencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruLowFrequencyContainer& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruMotionPredictionContainer& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruMovementControl& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruProfileAndSubprofile& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruSizeClass& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruSpecificExteriorLights& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfileAnimal& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfileBicyclist& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfileMotorcyclist& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfilePedestrian& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Wgs84Angle& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Wgs84AngleConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::Wgs84AngleValue& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::YawRate& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::YawRateConfidence& msg);
bsoncxx::document::value toBson(const etsi_its_vam_ts_msgs::msg::YawRateValue& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Accel& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::AccelStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::AccelWithCovariance& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::AccelWithCovarianceStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Inertia& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::InertiaStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Point& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Point32& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::PointStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Polygon& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::PolygonStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Pose& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Pose2D& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::PoseArray& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::PoseStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::PoseWithCovariance& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Quaternion& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::QuaternionStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Transform& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::TransformStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Twist& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::TwistStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::TwistWithCovariance& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::TwistWithCovarianceStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Vector3& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Vector3Stamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::VelocityStamped& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::Wrench& msg);
bsoncxx::document::value toBson(const geometry_msgs::msg::WrenchStamped& msg);
bsoncxx::document::value toBson(const nav_msgs::msg::GridCells& msg);
bsoncxx::document::value toBson(const nav_msgs::msg::MapMetaData& msg);
bsoncxx::document::value toBson(const nav_msgs::msg::OccupancyGrid& msg);
bsoncxx::document::value toBson(const nav_msgs::msg::Odometry& msg);
bsoncxx::document::value toBson(const nav_msgs::msg::Path& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::EgoData& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::Object& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::ObjectClassification& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::ObjectList& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::ObjectReferencePoint& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::ObjectState& msg);
bsoncxx::document::value toBson(const perception_msgs::msg::ObjectStatePrediction& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::BatteryState& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::CameraInfo& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::ChannelFloat32& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::CompressedImage& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::FluidPressure& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::Illuminance& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::Image& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::Imu& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::JointState& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::Joy& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::JoyFeedback& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::JoyFeedbackArray& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::LaserEcho& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::LaserScan& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::MagneticField& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::MultiDOFJointState& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::MultiEchoLaserScan& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::NavSatFix& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::NavSatStatus& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::PointCloud& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::PointCloud2& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::PointField& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::Range& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::RegionOfInterest& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::RelativeHumidity& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::Temperature& msg);
bsoncxx::document::value toBson(const sensor_msgs::msg::TimeReference& msg);
bsoncxx::document::value toBson(const shape_msgs::msg::Mesh& msg);
bsoncxx::document::value toBson(const shape_msgs::msg::MeshTriangle& msg);
bsoncxx::document::value toBson(const shape_msgs::msg::Plane& msg);
bsoncxx::document::value toBson(const shape_msgs::msg::SolidPrimitive& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Bool& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Byte& msg);
bsoncxx::document::value toBson(const std_msgs::msg::ByteMultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Char& msg);
bsoncxx::document::value toBson(const std_msgs::msg::ColorRGBA& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Empty& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Float32& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Float32MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Float64& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Float64MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Header& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int16& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int16MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int32& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int32MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int64& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int64MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int8& msg);
bsoncxx::document::value toBson(const std_msgs::msg::Int8MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::MultiArrayDimension& msg);
bsoncxx::document::value toBson(const std_msgs::msg::MultiArrayLayout& msg);
bsoncxx::document::value toBson(const std_msgs::msg::String& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt16& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt16MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt32& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt32MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt64& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt64MultiArray& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt8& msg);
bsoncxx::document::value toBson(const std_msgs::msg::UInt8MultiArray& msg);
bsoncxx::document::value toBson(const stereo_msgs::msg::DisparityImage& msg);
bsoncxx::document::value toBson(const tf2_msgs::msg::TF2Error& msg);
bsoncxx::document::value toBson(const tf2_msgs::msg::TFMessage& msg);
bsoncxx::document::value toBson(const trajectory_msgs::msg::JointTrajectory& msg);
bsoncxx::document::value toBson(const trajectory_msgs::msg::JointTrajectoryPoint& msg);
bsoncxx::document::value toBson(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg);
bsoncxx::document::value toBson(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::BoundingBox2D& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::BoundingBox2DArray& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::BoundingBox3D& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::BoundingBox3DArray& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Classification& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Detection2D& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Detection2DArray& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Detection3D& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Detection3DArray& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::LabelInfo& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::ObjectHypothesis& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::ObjectHypothesisWithPose& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Point2D& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::Pose2D& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::VisionClass& msg);
bsoncxx::document::value toBson(const vision_msgs::msg::VisionInfo& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::ImageMarker& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::InteractiveMarker& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::InteractiveMarkerControl& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::InteractiveMarkerFeedback& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::InteractiveMarkerInit& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::InteractiveMarkerPose& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::InteractiveMarkerUpdate& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::Marker& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::MarkerArray& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::MenuEntry& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::MeshFile& msg);
bsoncxx::document::value toBson(const visualization_msgs::msg::UVCoordinate& msg);
bsoncxx::document::value toBson(const event_detector_db_recording_msgs::msg::ImageFile& msg);
bsoncxx::document::value toBson(const event_detector_db_recording_msgs::msg::PointCloudFile& msg);

}  // namespace event_detector_db_recording_plugin

#include <event_detector_db_recording_plugin/toBson.tpp>
