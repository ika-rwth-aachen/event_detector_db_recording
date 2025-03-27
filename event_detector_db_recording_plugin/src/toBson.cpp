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

#include <chrono>

#include "event_detector_db_recording_plugin/toBson.hpp"

namespace event_detector_db_recording_plugin {

using namespace bsoncxx::document;
using namespace bsoncxx::builder::stream;

// clang-format off
bsoncxx::types::b_double toBson(const float d) {
  return bsoncxx::types::b_double{static_cast<double>(d)};
}

bsoncxx::types::b_double toBson(const double d) {
  return bsoncxx::types::b_double{d};
}

bsoncxx::types::b_string toBson(const std::string& s) {
  return bsoncxx::types::b_string{s};
}

bsoncxx::types::b_bool toBson(const bool b) {
  return bsoncxx::types::b_bool{b};
}

bsoncxx::types::b_date toBson(const rclcpp::Time& time) {
  long long ms = static_cast<long long>(time.nanoseconds() / 1000000);
  return bsoncxx::types::b_date{std::chrono::milliseconds{ms}};
}

bsoncxx::types::b_double toBson(const rclcpp::Duration& duration) {
  return bsoncxx::types::b_double{duration.seconds()};
}

bsoncxx::types::b_int32 toBson(const signed char i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int32 toBson(const unsigned char i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int32 toBson(const short int i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int32 toBson(const unsigned short int i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int32 toBson(const int i) {
  return bsoncxx::types::b_int32{i};
}

bsoncxx::types::b_int32 toBson(const unsigned int i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int32 toBson(const long int i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int32 toBson(const unsigned long int i) {
  return bsoncxx::types::b_int32{static_cast<int>(i)};
}

bsoncxx::types::b_int64 toBson(const long long i) {
  return bsoncxx::types::b_int64{i};
}

bsoncxx::types::b_int64 toBson(const unsigned long long int i) {
  return bsoncxx::types::b_int64{static_cast<long long int>(i)};
}

bsoncxx::types::b_oid toBson(const bsoncxx::oid& oid) {
  return bsoncxx::types::b_oid{oid};
}

value toBson(const nlohmann::json& j) {
  return bsoncxx::from_json(j.dump());
}

// =============================================================================

value toBson(const builtin_interfaces::msg::Duration& msg) {
  return document{}
    << "sec"     << toBson(msg.sec)     // int32
    << "nanosec" << toBson(msg.nanosec) // uint32
  << finalize;
}

value toBson(const builtin_interfaces::msg::Time& msg) {
  return document{}
    << "sec"     << toBson(msg.sec)     // int32
    << "nanosec" << toBson(msg.nanosec) // uint32
  << finalize;
}

value toBson(const diagnostic_msgs::msg::DiagnosticArray& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "status" << toBson(msg.status) // diagnostic_msgs/DiagnosticStatus[]
  << finalize;
}

value toBson(const diagnostic_msgs::msg::DiagnosticStatus& msg) {
  return document{}
    << "level"       << toBson(msg.level)       // byte
    << "name"        << toBson(msg.name)        // string
    << "message"     << toBson(msg.message)     // string
    << "hardware_id" << toBson(msg.hardware_id) // string
    << "values"      << toBson(msg.values)      // diagnostic_msgs/KeyValue[]
  << finalize;
}

value toBson(const diagnostic_msgs::msg::KeyValue& msg) {
  return document{}
    << "key"   << toBson(msg.key)   // string
    << "value" << toBson(msg.value) // string
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::AccelerationConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::AccelerationControl& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::Altitude& msg) {
  return document{}
    << "altitude_value"      << toBson(msg.altitude_value)      // etsi_its_cam_msgs/AltitudeValue
    << "altitude_confidence" << toBson(msg.altitude_confidence) // etsi_its_cam_msgs/AltitudeConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::AltitudeConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::AltitudeValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::BasicContainer& msg) {
  return document{}
    << "station_type"       << toBson(msg.station_type)       // etsi_its_cam_msgs/StationType
    << "reference_position" << toBson(msg.reference_position) // etsi_its_cam_msgs/ReferencePosition
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::BasicVehicleContainerHighFrequency& msg) {
  return document{}
    << "heading"                          << toBson(msg.heading)                          // etsi_its_cam_msgs/Heading
    << "speed"                            << toBson(msg.speed)                            // etsi_its_cam_msgs/Speed
    << "drive_direction"                  << toBson(msg.drive_direction)                  // etsi_its_cam_msgs/DriveDirection
    << "vehicle_length"                   << toBson(msg.vehicle_length)                   // etsi_its_cam_msgs/VehicleLength
    << "vehicle_width"                    << toBson(msg.vehicle_width)                    // etsi_its_cam_msgs/VehicleWidth
    << "longitudinal_acceleration"        << toBson(msg.longitudinal_acceleration)        // etsi_its_cam_msgs/LongitudinalAcceleration
    << "curvature"                        << toBson(msg.curvature)                        // etsi_its_cam_msgs/Curvature
    << "curvature_calculation_mode"       << toBson(msg.curvature_calculation_mode)       // etsi_its_cam_msgs/CurvatureCalculationMode
    << "yaw_rate"                         << toBson(msg.yaw_rate)                         // etsi_its_cam_msgs/YawRate
    << "acceleration_control"             << toBson(msg.acceleration_control)             // etsi_its_cam_msgs/AccelerationControl
    << "acceleration_control_is_present"  << toBson(msg.acceleration_control_is_present)  // bool
    << "lane_position"                    << toBson(msg.lane_position)                    // etsi_its_cam_msgs/LanePosition
    << "lane_position_is_present"         << toBson(msg.lane_position_is_present)         // bool
    << "steering_wheel_angle"             << toBson(msg.steering_wheel_angle)             // etsi_its_cam_msgs/SteeringWheelAngle
    << "steering_wheel_angle_is_present"  << toBson(msg.steering_wheel_angle_is_present)  // bool
    << "lateral_acceleration"             << toBson(msg.lateral_acceleration)             // etsi_its_cam_msgs/LateralAcceleration
    << "lateral_acceleration_is_present"  << toBson(msg.lateral_acceleration_is_present)  // bool
    << "vertical_acceleration"            << toBson(msg.vertical_acceleration)            // etsi_its_cam_msgs/VerticalAcceleration
    << "vertical_acceleration_is_present" << toBson(msg.vertical_acceleration_is_present) // bool
    << "performance_class"                << toBson(msg.performance_class)                // etsi_its_cam_msgs/PerformanceClass
    << "performance_class_is_present"     << toBson(msg.performance_class_is_present)     // bool
    << "cen_dsrc_tolling_zone"            << toBson(msg.cen_dsrc_tolling_zone)            // etsi_its_cam_msgs/CenDsrcTollingZone
    << "cen_dsrc_tolling_zone_is_present" << toBson(msg.cen_dsrc_tolling_zone_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::BasicVehicleContainerLowFrequency& msg) {
  return document{}
    << "vehicle_role"    << toBson(msg.vehicle_role)    // etsi_its_cam_msgs/VehicleRole
    << "exterior_lights" << toBson(msg.exterior_lights) // etsi_its_cam_msgs/ExteriorLights
    << "path_history"    << toBson(msg.path_history)    // etsi_its_cam_msgs/PathHistory
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CAM& msg) {
  return document{}
    << "header" << toBson(msg.header) // etsi_its_cam_msgs/ItsPduHeader
    << "cam"    << toBson(msg.cam)    // etsi_its_cam_msgs/CoopAwareness
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CamParameters& msg) {
  return document{}
    << "basic_container"                      << toBson(msg.basic_container)                      // etsi_its_cam_msgs/BasicContainer
    << "high_frequency_container"             << toBson(msg.high_frequency_container)             // etsi_its_cam_msgs/HighFrequencyContainer
    << "low_frequency_container"              << toBson(msg.low_frequency_container)              // etsi_its_cam_msgs/LowFrequencyContainer
    << "low_frequency_container_is_present"   << toBson(msg.low_frequency_container_is_present)   // bool
    << "special_vehicle_container"            << toBson(msg.special_vehicle_container)            // etsi_its_cam_msgs/SpecialVehicleContainer
    << "special_vehicle_container_is_present" << toBson(msg.special_vehicle_container_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CauseCode& msg) {
  return document{}
    << "cause_code"     << toBson(msg.cause_code)     // etsi_its_cam_msgs/CauseCodeType
    << "sub_cause_code" << toBson(msg.sub_cause_code) // etsi_its_cam_msgs/SubCauseCodeType
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CauseCodeType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CenDsrcTollingZone& msg) {
  return document{}
    << "protected_zone_latitude"             << toBson(msg.protected_zone_latitude)             // etsi_its_cam_msgs/Latitude
    << "protected_zone_longitude"            << toBson(msg.protected_zone_longitude)            // etsi_its_cam_msgs/Longitude
    << "cen_dsrc_tolling_zone_id"            << toBson(msg.cen_dsrc_tolling_zone_id)            // etsi_its_cam_msgs/CenDsrcTollingZoneID
    << "cen_dsrc_tolling_zone_id_is_present" << toBson(msg.cen_dsrc_tolling_zone_id_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CenDsrcTollingZoneID& msg) {
  return document{}
    << "value" << toBson(msg.value) // etsi_its_cam_msgs/ProtectedZoneID
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ClosedLanes& msg) {
  return document{}
    << "innerhard_shoulder_status"            << toBson(msg.innerhard_shoulder_status)            // etsi_its_cam_msgs/HardShoulderStatus
    << "innerhard_shoulder_status_is_present" << toBson(msg.innerhard_shoulder_status_is_present) // bool
    << "outerhard_shoulder_status"            << toBson(msg.outerhard_shoulder_status)            // etsi_its_cam_msgs/HardShoulderStatus
    << "outerhard_shoulder_status_is_present" << toBson(msg.outerhard_shoulder_status_is_present) // bool
    << "driving_lane_status"                  << toBson(msg.driving_lane_status)                  // etsi_its_cam_msgs/DrivingLaneStatus
    << "driving_lane_status_is_present"       << toBson(msg.driving_lane_status_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CoopAwareness& msg) {
  return document{}
    << "generation_delta_time" << toBson(msg.generation_delta_time) // etsi_its_cam_msgs/GenerationDeltaTime
    << "cam_parameters"        << toBson(msg.cam_parameters)        // etsi_its_cam_msgs/CamParameters
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::Curvature& msg) {
  return document{}
    << "curvature_value"      << toBson(msg.curvature_value)      // etsi_its_cam_msgs/CurvatureValue
    << "curvature_confidence" << toBson(msg.curvature_confidence) // etsi_its_cam_msgs/CurvatureConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CurvatureCalculationMode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CurvatureConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::CurvatureValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DangerousGoodsBasic& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DangerousGoodsContainer& msg) {
  return document{}
    << "dangerous_goods_basic" << toBson(msg.dangerous_goods_basic) // etsi_its_cam_msgs/DangerousGoodsBasic
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DeltaAltitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DeltaLatitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DeltaLongitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DeltaReferencePosition& msg) {
  return document{}
    << "delta_latitude"  << toBson(msg.delta_latitude)  // etsi_its_cam_msgs/DeltaLatitude
    << "delta_longitude" << toBson(msg.delta_longitude) // etsi_its_cam_msgs/DeltaLongitude
    << "delta_altitude"  << toBson(msg.delta_altitude)  // etsi_its_cam_msgs/DeltaAltitude
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DriveDirection& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::DrivingLaneStatus& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::EmbarkationStatus& msg) {
  return document{}
    << "value" << toBson(msg.value) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::EmergencyContainer& msg) {
  return document{}
    << "light_bar_siren_in_use"         << toBson(msg.light_bar_siren_in_use)         // etsi_its_cam_msgs/LightBarSirenInUse
    << "incident_indication"            << toBson(msg.incident_indication)            // etsi_its_cam_msgs/CauseCode
    << "incident_indication_is_present" << toBson(msg.incident_indication_is_present) // bool
    << "emergency_priority"             << toBson(msg.emergency_priority)             // etsi_its_cam_msgs/EmergencyPriority
    << "emergency_priority_is_present"  << toBson(msg.emergency_priority_is_present)  // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::EmergencyPriority& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ExteriorLights& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::GenerationDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::HardShoulderStatus& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::Heading& msg) {
  return document{}
    << "heading_value"      << toBson(msg.heading_value)      // etsi_its_cam_msgs/HeadingValue
    << "heading_confidence" << toBson(msg.heading_confidence) // etsi_its_cam_msgs/HeadingConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::HeadingConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::HeadingValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::HighFrequencyContainer& msg) {
  return document{}
    << "choice"                                 << toBson(msg.choice)                                 // uint8
    << "basic_vehicle_container_high_frequency" << toBson(msg.basic_vehicle_container_high_frequency) // etsi_its_cam_msgs/BasicVehicleContainerHighFrequency
    << "rsu_container_high_frequency"           << toBson(msg.rsu_container_high_frequency)           // etsi_its_cam_msgs/RSUContainerHighFrequency
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // uint8
    << "message_id"       << toBson(msg.message_id)       // uint8
    << "station_id"       << toBson(msg.station_id)       // etsi_its_cam_msgs/StationID
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LanePosition& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LateralAcceleration& msg) {
  return document{}
    << "lateral_acceleration_value"      << toBson(msg.lateral_acceleration_value)      // etsi_its_cam_msgs/LateralAccelerationValue
    << "lateral_acceleration_confidence" << toBson(msg.lateral_acceleration_confidence) // etsi_its_cam_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LateralAccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::Latitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LightBarSirenInUse& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::Longitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LongitudinalAcceleration& msg) {
  return document{}
    << "longitudinal_acceleration_value"      << toBson(msg.longitudinal_acceleration_value)      // etsi_its_cam_msgs/LongitudinalAccelerationValue
    << "longitudinal_acceleration_confidence" << toBson(msg.longitudinal_acceleration_confidence) // etsi_its_cam_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LongitudinalAccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::LowFrequencyContainer& msg) {
  return document{}
    << "choice"                                << toBson(msg.choice)                                // uint8
    << "basic_vehicle_container_low_frequency" << toBson(msg.basic_vehicle_container_low_frequency) // etsi_its_cam_msgs/BasicVehicleContainerLowFrequency
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PathDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PathHistory& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cam_msgs/PathPoint[]
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PathPoint& msg) {
  return document{}
    << "path_position"              << toBson(msg.path_position)              // etsi_its_cam_msgs/DeltaReferencePosition
    << "path_delta_time"            << toBson(msg.path_delta_time)            // etsi_its_cam_msgs/PathDeltaTime
    << "path_delta_time_is_present" << toBson(msg.path_delta_time_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PerformanceClass& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PosConfidenceEllipse& msg) {
  return document{}
    << "semi_major_confidence"  << toBson(msg.semi_major_confidence)  // etsi_its_cam_msgs/SemiAxisLength
    << "semi_minor_confidence"  << toBson(msg.semi_minor_confidence)  // etsi_its_cam_msgs/SemiAxisLength
    << "semi_major_orientation" << toBson(msg.semi_major_orientation) // etsi_its_cam_msgs/HeadingValue
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ProtectedCommunicationZone& msg) {
  return document{}
    << "protected_zone_type"              << toBson(msg.protected_zone_type)              // etsi_its_cam_msgs/ProtectedZoneType
    << "expiry_time"                      << toBson(msg.expiry_time)                      // etsi_its_cam_msgs/TimestampIts
    << "expiry_time_is_present"           << toBson(msg.expiry_time_is_present)           // bool
    << "protected_zone_latitude"          << toBson(msg.protected_zone_latitude)          // etsi_its_cam_msgs/Latitude
    << "protected_zone_longitude"         << toBson(msg.protected_zone_longitude)         // etsi_its_cam_msgs/Longitude
    << "protected_zone_radius"            << toBson(msg.protected_zone_radius)            // etsi_its_cam_msgs/ProtectedZoneRadius
    << "protected_zone_radius_is_present" << toBson(msg.protected_zone_radius_is_present) // bool
    << "protected_zone_id"                << toBson(msg.protected_zone_id)                // etsi_its_cam_msgs/ProtectedZoneID
    << "protected_zone_id_is_present"     << toBson(msg.protected_zone_id_is_present)     // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ProtectedCommunicationZonesRSU& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cam_msgs/ProtectedCommunicationZone[]
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ProtectedZoneID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ProtectedZoneRadius& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ProtectedZoneType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PtActivation& msg) {
  return document{}
    << "pt_activation_type" << toBson(msg.pt_activation_type) // etsi_its_cam_msgs/PtActivationType
    << "pt_activation_data" << toBson(msg.pt_activation_data) // etsi_its_cam_msgs/PtActivationData
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PtActivationData& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8[]
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PtActivationType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::PublicTransportContainer& msg) {
  return document{}
    << "embarkation_status"       << toBson(msg.embarkation_status)       // etsi_its_cam_msgs/EmbarkationStatus
    << "pt_activation"            << toBson(msg.pt_activation)            // etsi_its_cam_msgs/PtActivation
    << "pt_activation_is_present" << toBson(msg.pt_activation_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::RSUContainerHighFrequency& msg) {
  return document{}
    << "protected_communication_zones_rsu"            << toBson(msg.protected_communication_zones_rsu)            // etsi_its_cam_msgs/ProtectedCommunicationZonesRSU
    << "protected_communication_zones_rsu_is_present" << toBson(msg.protected_communication_zones_rsu_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::ReferencePosition& msg) {
  return document{}
    << "latitude"                    << toBson(msg.latitude)                    // etsi_its_cam_msgs/Latitude
    << "longitude"                   << toBson(msg.longitude)                   // etsi_its_cam_msgs/Longitude
    << "position_confidence_ellipse" << toBson(msg.position_confidence_ellipse) // etsi_its_cam_msgs/PosConfidenceEllipse
    << "altitude"                    << toBson(msg.altitude)                    // etsi_its_cam_msgs/Altitude
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::RescueContainer& msg) {
  return document{}
    << "light_bar_siren_in_use" << toBson(msg.light_bar_siren_in_use) // etsi_its_cam_msgs/LightBarSirenInUse
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::RoadWorksContainerBasic& msg) {
  return document{}
    << "roadworks_sub_cause_code"            << toBson(msg.roadworks_sub_cause_code)            // etsi_its_cam_msgs/RoadworksSubCauseCode
    << "roadworks_sub_cause_code_is_present" << toBson(msg.roadworks_sub_cause_code_is_present) // bool
    << "light_bar_siren_in_use"              << toBson(msg.light_bar_siren_in_use)              // etsi_its_cam_msgs/LightBarSirenInUse
    << "closed_lanes"                        << toBson(msg.closed_lanes)                        // etsi_its_cam_msgs/ClosedLanes
    << "closed_lanes_is_present"             << toBson(msg.closed_lanes_is_present)             // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::RoadworksSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SafetyCarContainer& msg) {
  return document{}
    << "light_bar_siren_in_use"         << toBson(msg.light_bar_siren_in_use)         // etsi_its_cam_msgs/LightBarSirenInUse
    << "incident_indication"            << toBson(msg.incident_indication)            // etsi_its_cam_msgs/CauseCode
    << "incident_indication_is_present" << toBson(msg.incident_indication_is_present) // bool
    << "traffic_rule"                   << toBson(msg.traffic_rule)                   // etsi_its_cam_msgs/TrafficRule
    << "traffic_rule_is_present"        << toBson(msg.traffic_rule_is_present)        // bool
    << "speed_limit"                    << toBson(msg.speed_limit)                    // etsi_its_cam_msgs/SpeedLimit
    << "speed_limit_is_present"         << toBson(msg.speed_limit_is_present)         // bool
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SemiAxisLength& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SpecialTransportContainer& msg) {
  return document{}
    << "special_transport_type" << toBson(msg.special_transport_type) // etsi_its_cam_msgs/SpecialTransportType
    << "light_bar_siren_in_use" << toBson(msg.light_bar_siren_in_use) // etsi_its_cam_msgs/LightBarSirenInUse
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SpecialTransportType& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SpecialVehicleContainer& msg) {
  return document{}
    << "choice"                      << toBson(msg.choice)                      // uint8
    << "public_transport_container"  << toBson(msg.public_transport_container)  // etsi_its_cam_msgs/PublicTransportContainer
    << "special_transport_container" << toBson(msg.special_transport_container) // etsi_its_cam_msgs/SpecialTransportContainer
    << "dangerous_goods_container"   << toBson(msg.dangerous_goods_container)   // etsi_its_cam_msgs/DangerousGoodsContainer
    << "road_works_container_basic"  << toBson(msg.road_works_container_basic)  // etsi_its_cam_msgs/RoadWorksContainerBasic
    << "rescue_container"            << toBson(msg.rescue_container)            // etsi_its_cam_msgs/RescueContainer
    << "emergency_container"         << toBson(msg.emergency_container)         // etsi_its_cam_msgs/EmergencyContainer
    << "safety_car_container"        << toBson(msg.safety_car_container)        // etsi_its_cam_msgs/SafetyCarContainer
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::Speed& msg) {
  return document{}
    << "speed_value"      << toBson(msg.speed_value)      // etsi_its_cam_msgs/SpeedValue
    << "speed_confidence" << toBson(msg.speed_confidence) // etsi_its_cam_msgs/SpeedConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SpeedConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SpeedLimit& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SpeedValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::StationID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::StationType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SteeringWheelAngle& msg) {
  return document{}
    << "steering_wheel_angle_value"      << toBson(msg.steering_wheel_angle_value)      // etsi_its_cam_msgs/SteeringWheelAngleValue
    << "steering_wheel_angle_confidence" << toBson(msg.steering_wheel_angle_confidence) // etsi_its_cam_msgs/SteeringWheelAngleConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SteeringWheelAngleConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SteeringWheelAngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::SubCauseCodeType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::TimestampIts& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint64
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::TrafficRule& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VehicleLength& msg) {
  return document{}
    << "vehicle_length_value"                 << toBson(msg.vehicle_length_value)                 // etsi_its_cam_msgs/VehicleLengthValue
    << "vehicle_length_confidence_indication" << toBson(msg.vehicle_length_confidence_indication) // etsi_its_cam_msgs/VehicleLengthConfidenceIndication
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VehicleLengthConfidenceIndication& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VehicleLengthValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VehicleRole& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VehicleWidth& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VerticalAcceleration& msg) {
  return document{}
    << "vertical_acceleration_value"      << toBson(msg.vertical_acceleration_value)      // etsi_its_cam_msgs/VerticalAccelerationValue
    << "vertical_acceleration_confidence" << toBson(msg.vertical_acceleration_confidence) // etsi_its_cam_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::VerticalAccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::YawRate& msg) {
  return document{}
    << "yaw_rate_value"      << toBson(msg.yaw_rate_value)      // etsi_its_cam_msgs/YawRateValue
    << "yaw_rate_confidence" << toBson(msg.yaw_rate_confidence) // etsi_its_cam_msgs/YawRateConfidence
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::YawRateConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_msgs::msg::YawRateValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationComponent& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cam_ts_msgs/AccelerationValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cam_ts_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationControl& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AccidentSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionAdhesionSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionExtremeWeatherConditionSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionPrecipitationSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AdverseWeatherConditionVisibilitySubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Altitude& msg) {
  return document{}
    << "altitude_value"      << toBson(msg.altitude_value)      // etsi_its_cam_ts_msgs/AltitudeValue
    << "altitude_confidence" << toBson(msg.altitude_confidence) // etsi_its_cam_ts_msgs/AltitudeConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AltitudeConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::AltitudeValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::BasicContainer& msg) {
  return document{}
    << "station_type"       << toBson(msg.station_type)       // etsi_its_cam_ts_msgs/TrafficParticipantType
    << "reference_position" << toBson(msg.reference_position) // etsi_its_cam_ts_msgs/ReferencePositionWithConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::BasicVehicleContainerHighFrequency& msg) {
  return document{}
    << "heading"                          << toBson(msg.heading)                          // etsi_its_cam_ts_msgs/Heading
    << "speed"                            << toBson(msg.speed)                            // etsi_its_cam_ts_msgs/Speed
    << "drive_direction"                  << toBson(msg.drive_direction)                  // etsi_its_cam_ts_msgs/DriveDirection
    << "vehicle_length"                   << toBson(msg.vehicle_length)                   // etsi_its_cam_ts_msgs/VehicleLength
    << "vehicle_width"                    << toBson(msg.vehicle_width)                    // etsi_its_cam_ts_msgs/VehicleWidth
    << "longitudinal_acceleration"        << toBson(msg.longitudinal_acceleration)        // etsi_its_cam_ts_msgs/AccelerationComponent
    << "curvature"                        << toBson(msg.curvature)                        // etsi_its_cam_ts_msgs/Curvature
    << "curvature_calculation_mode"       << toBson(msg.curvature_calculation_mode)       // etsi_its_cam_ts_msgs/CurvatureCalculationMode
    << "yaw_rate"                         << toBson(msg.yaw_rate)                         // etsi_its_cam_ts_msgs/YawRate
    << "acceleration_control"             << toBson(msg.acceleration_control)             // etsi_its_cam_ts_msgs/AccelerationControl
    << "acceleration_control_is_present"  << toBson(msg.acceleration_control_is_present)  // bool
    << "lane_position"                    << toBson(msg.lane_position)                    // etsi_its_cam_ts_msgs/LanePosition
    << "lane_position_is_present"         << toBson(msg.lane_position_is_present)         // bool
    << "steering_wheel_angle"             << toBson(msg.steering_wheel_angle)             // etsi_its_cam_ts_msgs/SteeringWheelAngle
    << "steering_wheel_angle_is_present"  << toBson(msg.steering_wheel_angle_is_present)  // bool
    << "lateral_acceleration"             << toBson(msg.lateral_acceleration)             // etsi_its_cam_ts_msgs/AccelerationComponent
    << "lateral_acceleration_is_present"  << toBson(msg.lateral_acceleration_is_present)  // bool
    << "vertical_acceleration"            << toBson(msg.vertical_acceleration)            // etsi_its_cam_ts_msgs/AccelerationComponent
    << "vertical_acceleration_is_present" << toBson(msg.vertical_acceleration_is_present) // bool
    << "performance_class"                << toBson(msg.performance_class)                // etsi_its_cam_ts_msgs/PerformanceClass
    << "performance_class_is_present"     << toBson(msg.performance_class_is_present)     // bool
    << "cen_dsrc_tolling_zone"            << toBson(msg.cen_dsrc_tolling_zone)            // etsi_its_cam_ts_msgs/CenDsrcTollingZone
    << "cen_dsrc_tolling_zone_is_present" << toBson(msg.cen_dsrc_tolling_zone_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::BasicVehicleContainerLowFrequency& msg) {
  return document{}
    << "vehicle_role"    << toBson(msg.vehicle_role)    // etsi_its_cam_ts_msgs/VehicleRole
    << "exterior_lights" << toBson(msg.exterior_lights) // etsi_its_cam_ts_msgs/ExteriorLights
    << "path_history"    << toBson(msg.path_history)    // etsi_its_cam_ts_msgs/Path
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CAM& msg) {
  return document{}
    << "header" << toBson(msg.header) // etsi_its_cam_ts_msgs/ItsPduHeader
    << "cam"    << toBson(msg.cam)    // etsi_its_cam_ts_msgs/CamPayload
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CamParameters& msg) {
  return document{}
    << "basic_container"                      << toBson(msg.basic_container)                      // etsi_its_cam_ts_msgs/BasicContainer
    << "high_frequency_container"             << toBson(msg.high_frequency_container)             // etsi_its_cam_ts_msgs/HighFrequencyContainer
    << "low_frequency_container"              << toBson(msg.low_frequency_container)              // etsi_its_cam_ts_msgs/LowFrequencyContainer
    << "low_frequency_container_is_present"   << toBson(msg.low_frequency_container_is_present)   // bool
    << "special_vehicle_container"            << toBson(msg.special_vehicle_container)            // etsi_its_cam_ts_msgs/SpecialVehicleContainer
    << "special_vehicle_container_is_present" << toBson(msg.special_vehicle_container_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CamPayload& msg) {
  return document{}
    << "generation_delta_time" << toBson(msg.generation_delta_time) // etsi_its_cam_ts_msgs/GenerationDeltaTime
    << "cam_parameters"        << toBson(msg.cam_parameters)        // etsi_its_cam_ts_msgs/CamParameters
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CauseCodeChoice& msg) {
  return document{}
    << "choice"                                                << toBson(msg.choice)                                                // uint8
    << "reserved0"                                             << toBson(msg.reserved0)                                             // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "traffic_condition1"                                    << toBson(msg.traffic_condition1)                                    // etsi_its_cam_ts_msgs/TrafficConditionSubCauseCode
    << "accident2"                                             << toBson(msg.accident2)                                             // etsi_its_cam_ts_msgs/AccidentSubCauseCode
    << "roadworks3"                                            << toBson(msg.roadworks3)                                            // etsi_its_cam_ts_msgs/RoadworksSubCauseCode
    << "reserved4"                                             << toBson(msg.reserved4)                                             // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "impassability5"                                        << toBson(msg.impassability5)                                        // etsi_its_cam_ts_msgs/ImpassabilitySubCauseCode
    << "adverse_weather_condition_adhesion6"                   << toBson(msg.adverse_weather_condition_adhesion6)                   // etsi_its_cam_ts_msgs/AdverseWeatherConditionAdhesionSubCauseCode
    << "aquaplaning7"                                          << toBson(msg.aquaplaning7)                                          // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved8"                                             << toBson(msg.reserved8)                                             // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "hazardous_location_surface_condition9"                 << toBson(msg.hazardous_location_surface_condition9)                 // etsi_its_cam_ts_msgs/HazardousLocationSurfaceConditionSubCauseCode
    << "hazardous_location_obstacle_on_the_road10"             << toBson(msg.hazardous_location_obstacle_on_the_road10)             // etsi_its_cam_ts_msgs/HazardousLocationObstacleOnTheRoadSubCauseCode
    << "hazardous_location_animal_on_the_road11"               << toBson(msg.hazardous_location_animal_on_the_road11)               // etsi_its_cam_ts_msgs/HazardousLocationAnimalOnTheRoadSubCauseCode
    << "human_presence_on_the_road12"                          << toBson(msg.human_presence_on_the_road12)                          // etsi_its_cam_ts_msgs/HumanPresenceOnTheRoadSubCauseCode
    << "reserved13"                                            << toBson(msg.reserved13)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "wrong_way_driving14"                                   << toBson(msg.wrong_way_driving14)                                   // etsi_its_cam_ts_msgs/WrongWayDrivingSubCauseCode
    << "rescue_and_recovery_work_in_progress15"                << toBson(msg.rescue_and_recovery_work_in_progress15)                // etsi_its_cam_ts_msgs/RescueAndRecoveryWorkInProgressSubCauseCode
    << "reserved16"                                            << toBson(msg.reserved16)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "adverse_weather_condition_extreme_weather_condition17" << toBson(msg.adverse_weather_condition_extreme_weather_condition17) // etsi_its_cam_ts_msgs/AdverseWeatherConditionExtremeWeatherConditionSubCauseCode
    << "adverse_weather_condition_visibility18"                << toBson(msg.adverse_weather_condition_visibility18)                // etsi_its_cam_ts_msgs/AdverseWeatherConditionVisibilitySubCauseCode
    << "adverse_weather_condition_precipitation19"             << toBson(msg.adverse_weather_condition_precipitation19)             // etsi_its_cam_ts_msgs/AdverseWeatherConditionPrecipitationSubCauseCode
    << "violence20"                                            << toBson(msg.violence20)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved21"                                            << toBson(msg.reserved21)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved22"                                            << toBson(msg.reserved22)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved23"                                            << toBson(msg.reserved23)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved24"                                            << toBson(msg.reserved24)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved25"                                            << toBson(msg.reserved25)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "slow_vehicle26"                                        << toBson(msg.slow_vehicle26)                                        // etsi_its_cam_ts_msgs/SlowVehicleSubCauseCode
    << "dangerous_end_of_queue27"                              << toBson(msg.dangerous_end_of_queue27)                              // etsi_its_cam_ts_msgs/DangerousEndOfQueueSubCauseCode
    << "public_transport_vehicle_approaching28"                << toBson(msg.public_transport_vehicle_approaching28)                // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved29"                                            << toBson(msg.reserved29)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved30"                                            << toBson(msg.reserved30)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved31"                                            << toBson(msg.reserved31)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved32"                                            << toBson(msg.reserved32)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved33"                                            << toBson(msg.reserved33)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved34"                                            << toBson(msg.reserved34)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved35"                                            << toBson(msg.reserved35)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved36"                                            << toBson(msg.reserved36)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved37"                                            << toBson(msg.reserved37)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved38"                                            << toBson(msg.reserved38)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved39"                                            << toBson(msg.reserved39)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved40"                                            << toBson(msg.reserved40)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved41"                                            << toBson(msg.reserved41)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved42"                                            << toBson(msg.reserved42)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved43"                                            << toBson(msg.reserved43)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved44"                                            << toBson(msg.reserved44)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved45"                                            << toBson(msg.reserved45)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved46"                                            << toBson(msg.reserved46)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved47"                                            << toBson(msg.reserved47)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved48"                                            << toBson(msg.reserved48)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved49"                                            << toBson(msg.reserved49)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved50"                                            << toBson(msg.reserved50)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved51"                                            << toBson(msg.reserved51)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved52"                                            << toBson(msg.reserved52)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved53"                                            << toBson(msg.reserved53)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved54"                                            << toBson(msg.reserved54)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved55"                                            << toBson(msg.reserved55)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved56"                                            << toBson(msg.reserved56)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved57"                                            << toBson(msg.reserved57)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved58"                                            << toBson(msg.reserved58)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved59"                                            << toBson(msg.reserved59)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved60"                                            << toBson(msg.reserved60)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved61"                                            << toBson(msg.reserved61)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved62"                                            << toBson(msg.reserved62)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved63"                                            << toBson(msg.reserved63)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved64"                                            << toBson(msg.reserved64)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved65"                                            << toBson(msg.reserved65)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved66"                                            << toBson(msg.reserved66)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved67"                                            << toBson(msg.reserved67)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved68"                                            << toBson(msg.reserved68)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved69"                                            << toBson(msg.reserved69)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved70"                                            << toBson(msg.reserved70)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved71"                                            << toBson(msg.reserved71)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved72"                                            << toBson(msg.reserved72)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved73"                                            << toBson(msg.reserved73)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved74"                                            << toBson(msg.reserved74)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved75"                                            << toBson(msg.reserved75)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved76"                                            << toBson(msg.reserved76)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved77"                                            << toBson(msg.reserved77)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved78"                                            << toBson(msg.reserved78)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved79"                                            << toBson(msg.reserved79)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved80"                                            << toBson(msg.reserved80)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved81"                                            << toBson(msg.reserved81)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved82"                                            << toBson(msg.reserved82)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved83"                                            << toBson(msg.reserved83)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved84"                                            << toBson(msg.reserved84)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved85"                                            << toBson(msg.reserved85)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved86"                                            << toBson(msg.reserved86)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved87"                                            << toBson(msg.reserved87)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved88"                                            << toBson(msg.reserved88)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved89"                                            << toBson(msg.reserved89)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved90"                                            << toBson(msg.reserved90)                                            // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "vehicle_breakdown91"                                   << toBson(msg.vehicle_breakdown91)                                   // etsi_its_cam_ts_msgs/VehicleBreakdownSubCauseCode
    << "post_crash92"                                          << toBson(msg.post_crash92)                                          // etsi_its_cam_ts_msgs/PostCrashSubCauseCode
    << "human_problem93"                                       << toBson(msg.human_problem93)                                       // etsi_its_cam_ts_msgs/HumanProblemSubCauseCode
    << "stationary_vehicle94"                                  << toBson(msg.stationary_vehicle94)                                  // etsi_its_cam_ts_msgs/StationaryVehicleSubCauseCode
    << "emergency_vehicle_approaching95"                       << toBson(msg.emergency_vehicle_approaching95)                       // etsi_its_cam_ts_msgs/EmergencyVehicleApproachingSubCauseCode
    << "hazardous_location_dangerous_curve96"                  << toBson(msg.hazardous_location_dangerous_curve96)                  // etsi_its_cam_ts_msgs/HazardousLocationDangerousCurveSubCauseCode
    << "collision_risk97"                                      << toBson(msg.collision_risk97)                                      // etsi_its_cam_ts_msgs/CollisionRiskSubCauseCode
    << "signal_violation98"                                    << toBson(msg.signal_violation98)                                    // etsi_its_cam_ts_msgs/SignalViolationSubCauseCode
    << "dangerous_situation99"                                 << toBson(msg.dangerous_situation99)                                 // etsi_its_cam_ts_msgs/DangerousSituationSubCauseCode
    << "railway_level_crossing100"                             << toBson(msg.railway_level_crossing100)                             // etsi_its_cam_ts_msgs/RailwayLevelCrossingSubCauseCode
    << "reserved101"                                           << toBson(msg.reserved101)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved102"                                           << toBson(msg.reserved102)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved103"                                           << toBson(msg.reserved103)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved104"                                           << toBson(msg.reserved104)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved105"                                           << toBson(msg.reserved105)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved106"                                           << toBson(msg.reserved106)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved107"                                           << toBson(msg.reserved107)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved108"                                           << toBson(msg.reserved108)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved109"                                           << toBson(msg.reserved109)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved110"                                           << toBson(msg.reserved110)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved111"                                           << toBson(msg.reserved111)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved112"                                           << toBson(msg.reserved112)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved113"                                           << toBson(msg.reserved113)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved114"                                           << toBson(msg.reserved114)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved115"                                           << toBson(msg.reserved115)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved116"                                           << toBson(msg.reserved116)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved117"                                           << toBson(msg.reserved117)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved118"                                           << toBson(msg.reserved118)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved119"                                           << toBson(msg.reserved119)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved120"                                           << toBson(msg.reserved120)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved121"                                           << toBson(msg.reserved121)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved122"                                           << toBson(msg.reserved122)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved123"                                           << toBson(msg.reserved123)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved124"                                           << toBson(msg.reserved124)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved125"                                           << toBson(msg.reserved125)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved126"                                           << toBson(msg.reserved126)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved127"                                           << toBson(msg.reserved127)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
    << "reserved128"                                           << toBson(msg.reserved128)                                           // etsi_its_cam_ts_msgs/SubCauseCodeType
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CauseCodeV2& msg) {
  return document{}
    << "cc_and_scc" << toBson(msg.cc_and_scc) // etsi_its_cam_ts_msgs/CauseCodeChoice
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CenDsrcTollingZone& msg) {
  return document{}
    << "protected_zone_latitude"             << toBson(msg.protected_zone_latitude)             // etsi_its_cam_ts_msgs/Latitude
    << "protected_zone_longitude"            << toBson(msg.protected_zone_longitude)            // etsi_its_cam_ts_msgs/Longitude
    << "cen_dsrc_tolling_zone_id"            << toBson(msg.cen_dsrc_tolling_zone_id)            // etsi_its_cam_ts_msgs/ProtectedZoneId
    << "cen_dsrc_tolling_zone_id_is_present" << toBson(msg.cen_dsrc_tolling_zone_id_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ClosedLanes& msg) {
  return document{}
    << "innerhard_shoulder_status"            << toBson(msg.innerhard_shoulder_status)            // etsi_its_cam_ts_msgs/HardShoulderStatus
    << "innerhard_shoulder_status_is_present" << toBson(msg.innerhard_shoulder_status_is_present) // bool
    << "outerhard_shoulder_status"            << toBson(msg.outerhard_shoulder_status)            // etsi_its_cam_ts_msgs/HardShoulderStatus
    << "outerhard_shoulder_status_is_present" << toBson(msg.outerhard_shoulder_status_is_present) // bool
    << "driving_lane_status"                  << toBson(msg.driving_lane_status)                  // etsi_its_cam_ts_msgs/DrivingLaneStatus
    << "driving_lane_status_is_present"       << toBson(msg.driving_lane_status_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CollisionRiskSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Curvature& msg) {
  return document{}
    << "curvature_value"      << toBson(msg.curvature_value)      // etsi_its_cam_ts_msgs/CurvatureValue
    << "curvature_confidence" << toBson(msg.curvature_confidence) // etsi_its_cam_ts_msgs/CurvatureConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CurvatureCalculationMode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CurvatureConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::CurvatureValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DangerousEndOfQueueSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DangerousGoodsBasic& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DangerousGoodsContainer& msg) {
  return document{}
    << "dangerous_goods_basic" << toBson(msg.dangerous_goods_basic) // etsi_its_cam_ts_msgs/DangerousGoodsBasic
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DangerousSituationSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DeltaAltitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DeltaLatitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DeltaLongitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DeltaReferencePosition& msg) {
  return document{}
    << "delta_latitude"  << toBson(msg.delta_latitude)  // etsi_its_cam_ts_msgs/DeltaLatitude
    << "delta_longitude" << toBson(msg.delta_longitude) // etsi_its_cam_ts_msgs/DeltaLongitude
    << "delta_altitude"  << toBson(msg.delta_altitude)  // etsi_its_cam_ts_msgs/DeltaAltitude
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DriveDirection& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::DrivingLaneStatus& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::EmbarkationStatus& msg) {
  return document{}
    << "value" << toBson(msg.value) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::EmergencyContainer& msg) {
  return document{}
    << "light_bar_siren_in_use"         << toBson(msg.light_bar_siren_in_use)         // etsi_its_cam_ts_msgs/LightBarSirenInUse
    << "incident_indication"            << toBson(msg.incident_indication)            // etsi_its_cam_ts_msgs/CauseCodeV2
    << "incident_indication_is_present" << toBson(msg.incident_indication_is_present) // bool
    << "emergency_priority"             << toBson(msg.emergency_priority)             // etsi_its_cam_ts_msgs/EmergencyPriority
    << "emergency_priority_is_present"  << toBson(msg.emergency_priority_is_present)  // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::EmergencyPriority& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::EmergencyVehicleApproachingSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ExteriorLights& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::GenerationDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HardShoulderStatus& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationAnimalOnTheRoadSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationDangerousCurveSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationObstacleOnTheRoadSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HazardousLocationSurfaceConditionSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Heading& msg) {
  return document{}
    << "heading_value"      << toBson(msg.heading_value)      // etsi_its_cam_ts_msgs/HeadingValue
    << "heading_confidence" << toBson(msg.heading_confidence) // etsi_its_cam_ts_msgs/HeadingConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HeadingConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HeadingValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HighFrequencyContainer& msg) {
  return document{}
    << "choice"                                 << toBson(msg.choice)                                 // uint8
    << "basic_vehicle_container_high_frequency" << toBson(msg.basic_vehicle_container_high_frequency) // etsi_its_cam_ts_msgs/BasicVehicleContainerHighFrequency
    << "rsu_container_high_frequency"           << toBson(msg.rsu_container_high_frequency)           // etsi_its_cam_ts_msgs/RSUContainerHighFrequency
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HumanPresenceOnTheRoadSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::HumanProblemSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ImpassabilitySubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // etsi_its_cam_ts_msgs/OrdinalNumber1B
    << "message_id"       << toBson(msg.message_id)       // etsi_its_cam_ts_msgs/MessageId
    << "station_id"       << toBson(msg.station_id)       // etsi_its_cam_ts_msgs/StationId
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::LanePosition& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Latitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::LightBarSirenInUse& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Longitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::LowFrequencyContainer& msg) {
  return document{}
    << "choice"                                << toBson(msg.choice)                                // uint8
    << "basic_vehicle_container_low_frequency" << toBson(msg.basic_vehicle_container_low_frequency) // etsi_its_cam_ts_msgs/BasicVehicleContainerLowFrequency
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::MessageId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::OrdinalNumber1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Path& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cam_ts_msgs/PathPoint[]
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PathDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PathPoint& msg) {
  return document{}
    << "path_position"              << toBson(msg.path_position)              // etsi_its_cam_ts_msgs/DeltaReferencePosition
    << "path_delta_time"            << toBson(msg.path_delta_time)            // etsi_its_cam_ts_msgs/PathDeltaTime
    << "path_delta_time_is_present" << toBson(msg.path_delta_time_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PerformanceClass& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PositionConfidenceEllipse& msg) {
  return document{}
    << "semi_major_axis_length"      << toBson(msg.semi_major_axis_length)      // etsi_its_cam_ts_msgs/SemiAxisLength
    << "semi_minor_axis_length"      << toBson(msg.semi_minor_axis_length)      // etsi_its_cam_ts_msgs/SemiAxisLength
    << "semi_major_axis_orientation" << toBson(msg.semi_major_axis_orientation) // etsi_its_cam_ts_msgs/Wgs84AngleValue
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PostCrashSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedCommunicationZone& msg) {
  return document{}
    << "protected_zone_type"              << toBson(msg.protected_zone_type)              // etsi_its_cam_ts_msgs/ProtectedZoneType
    << "expiry_time"                      << toBson(msg.expiry_time)                      // etsi_its_cam_ts_msgs/TimestampIts
    << "expiry_time_is_present"           << toBson(msg.expiry_time_is_present)           // bool
    << "protected_zone_latitude"          << toBson(msg.protected_zone_latitude)          // etsi_its_cam_ts_msgs/Latitude
    << "protected_zone_longitude"         << toBson(msg.protected_zone_longitude)         // etsi_its_cam_ts_msgs/Longitude
    << "protected_zone_radius"            << toBson(msg.protected_zone_radius)            // etsi_its_cam_ts_msgs/ProtectedZoneRadius
    << "protected_zone_radius_is_present" << toBson(msg.protected_zone_radius_is_present) // bool
    << "protected_zone_id"                << toBson(msg.protected_zone_id)                // etsi_its_cam_ts_msgs/ProtectedZoneId
    << "protected_zone_id_is_present"     << toBson(msg.protected_zone_id_is_present)     // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedCommunicationZonesRSU& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cam_ts_msgs/ProtectedCommunicationZone[]
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedZoneId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedZoneRadius& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ProtectedZoneType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PtActivation& msg) {
  return document{}
    << "pt_activation_type" << toBson(msg.pt_activation_type) // etsi_its_cam_ts_msgs/PtActivationType
    << "pt_activation_data" << toBson(msg.pt_activation_data) // etsi_its_cam_ts_msgs/PtActivationData
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PtActivationData& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8[]
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PtActivationType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::PublicTransportContainer& msg) {
  return document{}
    << "embarkation_status"       << toBson(msg.embarkation_status)       // etsi_its_cam_ts_msgs/EmbarkationStatus
    << "pt_activation"            << toBson(msg.pt_activation)            // etsi_its_cam_ts_msgs/PtActivation
    << "pt_activation_is_present" << toBson(msg.pt_activation_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::RSUContainerHighFrequency& msg) {
  return document{}
    << "protected_communication_zones_rsu"            << toBson(msg.protected_communication_zones_rsu)            // etsi_its_cam_ts_msgs/ProtectedCommunicationZonesRSU
    << "protected_communication_zones_rsu_is_present" << toBson(msg.protected_communication_zones_rsu_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::RailwayLevelCrossingSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::ReferencePositionWithConfidence& msg) {
  return document{}
    << "latitude"                    << toBson(msg.latitude)                    // etsi_its_cam_ts_msgs/Latitude
    << "longitude"                   << toBson(msg.longitude)                   // etsi_its_cam_ts_msgs/Longitude
    << "position_confidence_ellipse" << toBson(msg.position_confidence_ellipse) // etsi_its_cam_ts_msgs/PositionConfidenceEllipse
    << "altitude"                    << toBson(msg.altitude)                    // etsi_its_cam_ts_msgs/Altitude
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::RescueAndRecoveryWorkInProgressSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::RescueContainer& msg) {
  return document{}
    << "light_bar_siren_in_use" << toBson(msg.light_bar_siren_in_use) // etsi_its_cam_ts_msgs/LightBarSirenInUse
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::RoadWorksContainerBasic& msg) {
  return document{}
    << "roadworks_sub_cause_code"            << toBson(msg.roadworks_sub_cause_code)            // etsi_its_cam_ts_msgs/RoadworksSubCauseCode
    << "roadworks_sub_cause_code_is_present" << toBson(msg.roadworks_sub_cause_code_is_present) // bool
    << "light_bar_siren_in_use"              << toBson(msg.light_bar_siren_in_use)              // etsi_its_cam_ts_msgs/LightBarSirenInUse
    << "closed_lanes"                        << toBson(msg.closed_lanes)                        // etsi_its_cam_ts_msgs/ClosedLanes
    << "closed_lanes_is_present"             << toBson(msg.closed_lanes_is_present)             // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::RoadworksSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SafetyCarContainer& msg) {
  return document{}
    << "light_bar_siren_in_use"         << toBson(msg.light_bar_siren_in_use)         // etsi_its_cam_ts_msgs/LightBarSirenInUse
    << "incident_indication"            << toBson(msg.incident_indication)            // etsi_its_cam_ts_msgs/CauseCodeV2
    << "incident_indication_is_present" << toBson(msg.incident_indication_is_present) // bool
    << "traffic_rule"                   << toBson(msg.traffic_rule)                   // etsi_its_cam_ts_msgs/TrafficRule
    << "traffic_rule_is_present"        << toBson(msg.traffic_rule_is_present)        // bool
    << "speed_limit"                    << toBson(msg.speed_limit)                    // etsi_its_cam_ts_msgs/SpeedLimit
    << "speed_limit_is_present"         << toBson(msg.speed_limit_is_present)         // bool
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SemiAxisLength& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SignalViolationSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SlowVehicleSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SpecialTransportContainer& msg) {
  return document{}
    << "special_transport_type" << toBson(msg.special_transport_type) // etsi_its_cam_ts_msgs/SpecialTransportType
    << "light_bar_siren_in_use" << toBson(msg.light_bar_siren_in_use) // etsi_its_cam_ts_msgs/LightBarSirenInUse
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SpecialTransportType& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SpecialVehicleContainer& msg) {
  return document{}
    << "choice"                      << toBson(msg.choice)                      // uint8
    << "public_transport_container"  << toBson(msg.public_transport_container)  // etsi_its_cam_ts_msgs/PublicTransportContainer
    << "special_transport_container" << toBson(msg.special_transport_container) // etsi_its_cam_ts_msgs/SpecialTransportContainer
    << "dangerous_goods_container"   << toBson(msg.dangerous_goods_container)   // etsi_its_cam_ts_msgs/DangerousGoodsContainer
    << "road_works_container_basic"  << toBson(msg.road_works_container_basic)  // etsi_its_cam_ts_msgs/RoadWorksContainerBasic
    << "rescue_container"            << toBson(msg.rescue_container)            // etsi_its_cam_ts_msgs/RescueContainer
    << "emergency_container"         << toBson(msg.emergency_container)         // etsi_its_cam_ts_msgs/EmergencyContainer
    << "safety_car_container"        << toBson(msg.safety_car_container)        // etsi_its_cam_ts_msgs/SafetyCarContainer
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Speed& msg) {
  return document{}
    << "speed_value"      << toBson(msg.speed_value)      // etsi_its_cam_ts_msgs/SpeedValue
    << "speed_confidence" << toBson(msg.speed_confidence) // etsi_its_cam_ts_msgs/SpeedConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SpeedConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SpeedLimit& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SpeedValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::StationId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::StationaryVehicleSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SteeringWheelAngle& msg) {
  return document{}
    << "steering_wheel_angle_value"      << toBson(msg.steering_wheel_angle_value)      // etsi_its_cam_ts_msgs/SteeringWheelAngleValue
    << "steering_wheel_angle_confidence" << toBson(msg.steering_wheel_angle_confidence) // etsi_its_cam_ts_msgs/SteeringWheelAngleConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SteeringWheelAngleConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SteeringWheelAngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::SubCauseCodeType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::TimestampIts& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint64
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::TrafficConditionSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::TrafficParticipantType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::TrafficRule& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::VehicleBreakdownSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::VehicleLength& msg) {
  return document{}
    << "vehicle_length_value"                 << toBson(msg.vehicle_length_value)                 // etsi_its_cam_ts_msgs/VehicleLengthValue
    << "vehicle_length_confidence_indication" << toBson(msg.vehicle_length_confidence_indication) // etsi_its_cam_ts_msgs/VehicleLengthConfidenceIndication
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::VehicleLengthConfidenceIndication& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::VehicleLengthValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::VehicleRole& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::VehicleWidth& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::Wgs84AngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::WrongWayDrivingSubCauseCode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::YawRate& msg) {
  return document{}
    << "yaw_rate_value"      << toBson(msg.yaw_rate_value)      // etsi_its_cam_ts_msgs/YawRateValue
    << "yaw_rate_confidence" << toBson(msg.yaw_rate_confidence) // etsi_its_cam_ts_msgs/YawRateConfidence
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::YawRateConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cam_ts_msgs::msg::YawRateValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Acceleration3dWithConfidence& msg) {
  return document{}
    << "choice"                 << toBson(msg.choice)                 // uint8
    << "polar_acceleration"     << toBson(msg.polar_acceleration)     // etsi_its_cpm_ts_msgs/AccelerationPolarWithZ
    << "cartesian_acceleration" << toBson(msg.cartesian_acceleration) // etsi_its_cpm_ts_msgs/AccelerationCartesian
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationCartesian& msg) {
  return document{}
    << "x_acceleration"            << toBson(msg.x_acceleration)            // etsi_its_cpm_ts_msgs/AccelerationComponent
    << "y_acceleration"            << toBson(msg.y_acceleration)            // etsi_its_cpm_ts_msgs/AccelerationComponent
    << "z_acceleration"            << toBson(msg.z_acceleration)            // etsi_its_cpm_ts_msgs/AccelerationComponent
    << "z_acceleration_is_present" << toBson(msg.z_acceleration_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationComponent& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/AccelerationValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationMagnitude& msg) {
  return document{}
    << "acceleration_magnitude_value" << toBson(msg.acceleration_magnitude_value) // etsi_its_cpm_ts_msgs/AccelerationMagnitudeValue
    << "acceleration_confidence"      << toBson(msg.acceleration_confidence)      // etsi_its_cpm_ts_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationMagnitudeValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationPolarWithZ& msg) {
  return document{}
    << "acceleration_magnitude"    << toBson(msg.acceleration_magnitude)    // etsi_its_cpm_ts_msgs/AccelerationMagnitude
    << "acceleration_direction"    << toBson(msg.acceleration_direction)    // etsi_its_cpm_ts_msgs/CartesianAngle
    << "z_acceleration"            << toBson(msg.z_acceleration)            // etsi_its_cpm_ts_msgs/AccelerationComponent
    << "z_acceleration_is_present" << toBson(msg.z_acceleration_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Altitude& msg) {
  return document{}
    << "altitude_value"      << toBson(msg.altitude_value)      // etsi_its_cpm_ts_msgs/AltitudeValue
    << "altitude_confidence" << toBson(msg.altitude_confidence) // etsi_its_cpm_ts_msgs/AltitudeConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AltitudeConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AltitudeValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AngleConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::AngularSpeedConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CardinalNumber1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CardinalNumber3b& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngle& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/AngleConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngularVelocityComponent& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/CartesianAngularVelocityComponentValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/AngularSpeedConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianAngularVelocityComponentValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinate& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinateLarge& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinateSmall& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianCoordinateWithConfidence& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/CartesianCoordinateLarge
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/CoordinateConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianPosition3d& msg) {
  return document{}
    << "x_coordinate"            << toBson(msg.x_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinate
    << "y_coordinate"            << toBson(msg.y_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinate
    << "z_coordinate"            << toBson(msg.z_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinate
    << "z_coordinate_is_present" << toBson(msg.z_coordinate_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CartesianPosition3dWithConfidence& msg) {
  return document{}
    << "x_coordinate"            << toBson(msg.x_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinateWithConfidence
    << "y_coordinate"            << toBson(msg.y_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinateWithConfidence
    << "z_coordinate"            << toBson(msg.z_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinateWithConfidence
    << "z_coordinate_is_present" << toBson(msg.z_coordinate_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CircularShape& msg) {
  return document{}
    << "shape_reference_point"            << toBson(msg.shape_reference_point)            // etsi_its_cpm_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present" << toBson(msg.shape_reference_point_is_present) // bool
    << "radius"                           << toBson(msg.radius)                           // etsi_its_cpm_ts_msgs/StandardLength12b
    << "height"                           << toBson(msg.height)                           // etsi_its_cpm_ts_msgs/StandardLength12b
    << "height_is_present"                << toBson(msg.height_is_present)                // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // etsi_its_cpm_ts_msgs/ItsPduHeader
    << "payload" << toBson(msg.payload) // etsi_its_cpm_ts_msgs/CpmPayload
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ConfidenceLevel& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ConstraintWrappedCpmContainers& msg) {
  return document{}
    << "value" << toBson(msg.value) // etsi_its_cpm_ts_msgs/WrappedCpmContainers
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CoordinateConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CorrelationCellValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CorrelationColumn& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/CorrelationCellValue[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CpmContainerId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::CpmPayload& msg) {
  return document{}
    << "management_container" << toBson(msg.management_container) // etsi_its_cpm_ts_msgs/ManagementContainer
    << "cpm_containers"       << toBson(msg.cpm_containers)       // etsi_its_cpm_ts_msgs/ConstraintWrappedCpmContainers
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::DeltaTimeMilliSecondSigned& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::EllipticalShape& msg) {
  return document{}
    << "shape_reference_point"            << toBson(msg.shape_reference_point)            // etsi_its_cpm_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present" << toBson(msg.shape_reference_point_is_present) // bool
    << "semi_major_axis_length"           << toBson(msg.semi_major_axis_length)           // etsi_its_cpm_ts_msgs/StandardLength12b
    << "semi_minor_axis_length"           << toBson(msg.semi_minor_axis_length)           // etsi_its_cpm_ts_msgs/StandardLength12b
    << "orientation"                      << toBson(msg.orientation)                      // etsi_its_cpm_ts_msgs/Wgs84AngleValue
    << "orientation_is_present"           << toBson(msg.orientation_is_present)           // bool
    << "height"                           << toBson(msg.height)                           // etsi_its_cpm_ts_msgs/StandardLength12b
    << "height_is_present"                << toBson(msg.height_is_present)                // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::EulerAnglesWithConfidence& msg) {
  return document{}
    << "z_angle"            << toBson(msg.z_angle)            // etsi_its_cpm_ts_msgs/CartesianAngle
    << "y_angle"            << toBson(msg.y_angle)            // etsi_its_cpm_ts_msgs/CartesianAngle
    << "y_angle_is_present" << toBson(msg.y_angle_is_present) // bool
    << "x_angle"            << toBson(msg.x_angle)            // etsi_its_cpm_ts_msgs/CartesianAngle
    << "x_angle_is_present" << toBson(msg.x_angle_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::HeadingValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Identifier1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Identifier2B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::IntersectionReferenceId& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_cpm_ts_msgs/Identifier2B
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_cpm_ts_msgs/Identifier2B
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // etsi_its_cpm_ts_msgs/OrdinalNumber1B
    << "message_id"       << toBson(msg.message_id)       // etsi_its_cpm_ts_msgs/MessageId
    << "station_id"       << toBson(msg.station_id)       // etsi_its_cpm_ts_msgs/StationId
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Latitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Longitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::LongitudinalLanePosition& msg) {
  return document{}
    << "longitudinal_lane_position_value"      << toBson(msg.longitudinal_lane_position_value)      // etsi_its_cpm_ts_msgs/LongitudinalLanePositionValue
    << "longitudinal_lane_position_confidence" << toBson(msg.longitudinal_lane_position_confidence) // etsi_its_cpm_ts_msgs/LongitudinalLanePositionConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::LongitudinalLanePositionConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::LongitudinalLanePositionValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::LowerTriangularPositiveSemidefiniteMatrices& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/LowerTriangularPositiveSemidefiniteMatrix[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::LowerTriangularPositiveSemidefiniteMatrix& msg) {
  return document{}
    << "components_included_inthe_matrix" << toBson(msg.components_included_inthe_matrix) // etsi_its_cpm_ts_msgs/MatrixIncludedComponents
    << "matrix"                           << toBson(msg.matrix)                           // etsi_its_cpm_ts_msgs/LowerTriangularPositiveSemidefiniteMatrixColumns
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::LowerTriangularPositiveSemidefiniteMatrixColumns& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/CorrelationColumn[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ManagementContainer& msg) {
  return document{}
    << "reference_time"                << toBson(msg.reference_time)                // etsi_its_cpm_ts_msgs/TimestampIts
    << "reference_position"            << toBson(msg.reference_position)            // etsi_its_cpm_ts_msgs/ReferencePosition
    << "segmentation_info"             << toBson(msg.segmentation_info)             // etsi_its_cpm_ts_msgs/MessageSegmentationInfo
    << "segmentation_info_is_present"  << toBson(msg.segmentation_info_is_present)  // bool
    << "message_rate_range"            << toBson(msg.message_rate_range)            // etsi_its_cpm_ts_msgs/MessageRateRange
    << "message_rate_range_is_present" << toBson(msg.message_rate_range_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MapPosition& msg) {
  return document{}
    << "map_reference"                         << toBson(msg.map_reference)                         // etsi_its_cpm_ts_msgs/MapReference
    << "map_reference_is_present"              << toBson(msg.map_reference_is_present)              // bool
    << "lane_id"                               << toBson(msg.lane_id)                               // etsi_its_cpm_ts_msgs/Identifier1B
    << "lane_id_is_present"                    << toBson(msg.lane_id_is_present)                    // bool
    << "connection_id"                         << toBson(msg.connection_id)                         // etsi_its_cpm_ts_msgs/Identifier1B
    << "connection_id_is_present"              << toBson(msg.connection_id_is_present)              // bool
    << "longitudinal_lane_position"            << toBson(msg.longitudinal_lane_position)            // etsi_its_cpm_ts_msgs/LongitudinalLanePosition
    << "longitudinal_lane_position_is_present" << toBson(msg.longitudinal_lane_position_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MapReference& msg) {
  return document{}
    << "choice"       << toBson(msg.choice)       // uint8
    << "roadsegment"  << toBson(msg.roadsegment)  // etsi_its_cpm_ts_msgs/RoadSegmentReferenceId
    << "intersection" << toBson(msg.intersection) // etsi_its_cpm_ts_msgs/IntersectionReferenceId
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MatrixIncludedComponents& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MessageId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MessageRateHz& msg) {
  return document{}
    << "mantissa" << toBson(msg.mantissa) // uint8
    << "exponent" << toBson(msg.exponent) // int8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MessageRateRange& msg) {
  return document{}
    << "message_rate_min" << toBson(msg.message_rate_min) // etsi_its_cpm_ts_msgs/MessageRateHz
    << "message_rate_max" << toBson(msg.message_rate_max) // etsi_its_cpm_ts_msgs/MessageRateHz
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::MessageSegmentationInfo& msg) {
  return document{}
    << "total_msg_no" << toBson(msg.total_msg_no) // etsi_its_cpm_ts_msgs/CardinalNumber3b
    << "this_msg_no"  << toBson(msg.this_msg_no)  // etsi_its_cpm_ts_msgs/OrdinalNumber3b
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectClass& msg) {
  return document{}
    << "choice"            << toBson(msg.choice)            // uint8
    << "vehicle_sub_class" << toBson(msg.vehicle_sub_class) // etsi_its_cpm_ts_msgs/TrafficParticipantType
    << "vru_sub_class"     << toBson(msg.vru_sub_class)     // etsi_its_cpm_ts_msgs/VruProfileAndSubprofile
    << "group_sub_class"   << toBson(msg.group_sub_class)   // etsi_its_cpm_ts_msgs/VruClusterInformation
    << "other_sub_class"   << toBson(msg.other_sub_class)   // etsi_its_cpm_ts_msgs/OtherSubClass
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectClassDescription& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/ObjectClassWithConfidence[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectClassWithConfidence& msg) {
  return document{}
    << "object_class" << toBson(msg.object_class) // etsi_its_cpm_ts_msgs/ObjectClass
    << "confidence"   << toBson(msg.confidence)   // etsi_its_cpm_ts_msgs/ConfidenceLevel
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectDimension& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/ObjectDimensionValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/ObjectDimensionConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectDimensionConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectDimensionValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ObjectPerceptionQuality& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::OrdinalNumber1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::OrdinalNumber3b& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::OriginatingRsuContainer& msg) {
  return document{}
    << "map_reference"            << toBson(msg.map_reference)            // etsi_its_cpm_ts_msgs/MapReference
    << "map_reference_is_present" << toBson(msg.map_reference_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::OriginatingVehicleContainer& msg) {
  return document{}
    << "orientation_angle"           << toBson(msg.orientation_angle)           // etsi_its_cpm_ts_msgs/Wgs84Angle
    << "pitch_angle"                 << toBson(msg.pitch_angle)                 // etsi_its_cpm_ts_msgs/CartesianAngle
    << "pitch_angle_is_present"      << toBson(msg.pitch_angle_is_present)      // bool
    << "roll_angle"                  << toBson(msg.roll_angle)                  // etsi_its_cpm_ts_msgs/CartesianAngle
    << "roll_angle_is_present"       << toBson(msg.roll_angle_is_present)       // bool
    << "trailer_data_set"            << toBson(msg.trailer_data_set)            // etsi_its_cpm_ts_msgs/TrailerDataSet
    << "trailer_data_set_is_present" << toBson(msg.trailer_data_set_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::OtherSubClass& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObject& msg) {
  return document{}
    << "object_id"                                        << toBson(msg.object_id)                                        // etsi_its_cpm_ts_msgs/Identifier2B
    << "object_id_is_present"                             << toBson(msg.object_id_is_present)                             // bool
    << "measurement_delta_time"                           << toBson(msg.measurement_delta_time)                           // etsi_its_cpm_ts_msgs/DeltaTimeMilliSecondSigned
    << "position"                                         << toBson(msg.position)                                         // etsi_its_cpm_ts_msgs/CartesianPosition3dWithConfidence
    << "velocity"                                         << toBson(msg.velocity)                                         // etsi_its_cpm_ts_msgs/Velocity3dWithConfidence
    << "velocity_is_present"                              << toBson(msg.velocity_is_present)                              // bool
    << "acceleration"                                     << toBson(msg.acceleration)                                     // etsi_its_cpm_ts_msgs/Acceleration3dWithConfidence
    << "acceleration_is_present"                          << toBson(msg.acceleration_is_present)                          // bool
    << "angles"                                           << toBson(msg.angles)                                           // etsi_its_cpm_ts_msgs/EulerAnglesWithConfidence
    << "angles_is_present"                                << toBson(msg.angles_is_present)                                // bool
    << "z_angular_velocity"                               << toBson(msg.z_angular_velocity)                               // etsi_its_cpm_ts_msgs/CartesianAngularVelocityComponent
    << "z_angular_velocity_is_present"                    << toBson(msg.z_angular_velocity_is_present)                    // bool
    << "lower_triangular_correlation_matrices"            << toBson(msg.lower_triangular_correlation_matrices)            // etsi_its_cpm_ts_msgs/LowerTriangularPositiveSemidefiniteMatrices
    << "lower_triangular_correlation_matrices_is_present" << toBson(msg.lower_triangular_correlation_matrices_is_present) // bool
    << "object_dimension_z"                               << toBson(msg.object_dimension_z)                               // etsi_its_cpm_ts_msgs/ObjectDimension
    << "object_dimension_z_is_present"                    << toBson(msg.object_dimension_z_is_present)                    // bool
    << "object_dimension_y"                               << toBson(msg.object_dimension_y)                               // etsi_its_cpm_ts_msgs/ObjectDimension
    << "object_dimension_y_is_present"                    << toBson(msg.object_dimension_y_is_present)                    // bool
    << "object_dimension_x"                               << toBson(msg.object_dimension_x)                               // etsi_its_cpm_ts_msgs/ObjectDimension
    << "object_dimension_x_is_present"                    << toBson(msg.object_dimension_x_is_present)                    // bool
    << "object_age"                                       << toBson(msg.object_age)                                       // etsi_its_cpm_ts_msgs/DeltaTimeMilliSecondSigned
    << "object_age_is_present"                            << toBson(msg.object_age_is_present)                            // bool
    << "object_perception_quality"                        << toBson(msg.object_perception_quality)                        // etsi_its_cpm_ts_msgs/ObjectPerceptionQuality
    << "object_perception_quality_is_present"             << toBson(msg.object_perception_quality_is_present)             // bool
    << "sensor_id_list"                                   << toBson(msg.sensor_id_list)                                   // etsi_its_cpm_ts_msgs/SequenceOfIdentifier1B
    << "sensor_id_list_is_present"                        << toBson(msg.sensor_id_list_is_present)                        // bool
    << "classification"                                   << toBson(msg.classification)                                   // etsi_its_cpm_ts_msgs/ObjectClassDescription
    << "classification_is_present"                        << toBson(msg.classification_is_present)                        // bool
    << "map_position"                                     << toBson(msg.map_position)                                     // etsi_its_cpm_ts_msgs/MapPosition
    << "map_position_is_present"                          << toBson(msg.map_position_is_present)                          // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObjectContainer& msg) {
  return document{}
    << "number_of_perceived_objects" << toBson(msg.number_of_perceived_objects) // etsi_its_cpm_ts_msgs/CardinalNumber1B
    << "perceived_objects"           << toBson(msg.perceived_objects)           // etsi_its_cpm_ts_msgs/PerceivedObjects
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObjectIds& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/Identifier2B[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PerceivedObjects& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/PerceivedObject[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PerceptionRegion& msg) {
  return document{}
    << "measurement_delta_time"                 << toBson(msg.measurement_delta_time)                 // etsi_its_cpm_ts_msgs/DeltaTimeMilliSecondSigned
    << "perception_region_confidence"           << toBson(msg.perception_region_confidence)           // etsi_its_cpm_ts_msgs/ConfidenceLevel
    << "perception_region_shape"                << toBson(msg.perception_region_shape)                // etsi_its_cpm_ts_msgs/Shape
    << "shadowing_applies"                      << toBson(msg.shadowing_applies)                      // bool
    << "sensor_id_list"                         << toBson(msg.sensor_id_list)                         // etsi_its_cpm_ts_msgs/SequenceOfIdentifier1B
    << "sensor_id_list_is_present"              << toBson(msg.sensor_id_list_is_present)              // bool
    << "number_of_perceived_objects"            << toBson(msg.number_of_perceived_objects)            // etsi_its_cpm_ts_msgs/CardinalNumber1B
    << "number_of_perceived_objects_is_present" << toBson(msg.number_of_perceived_objects_is_present) // bool
    << "perceived_object_ids"                   << toBson(msg.perceived_object_ids)                   // etsi_its_cpm_ts_msgs/PerceivedObjectIds
    << "perceived_object_ids_is_present"        << toBson(msg.perceived_object_ids_is_present)        // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PerceptionRegionContainer& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/PerceptionRegion[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PolygonalShape& msg) {
  return document{}
    << "shape_reference_point"            << toBson(msg.shape_reference_point)            // etsi_its_cpm_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present" << toBson(msg.shape_reference_point_is_present) // bool
    << "polygon"                          << toBson(msg.polygon)                          // etsi_its_cpm_ts_msgs/SequenceOfCartesianPosition3d
    << "height"                           << toBson(msg.height)                           // etsi_its_cpm_ts_msgs/StandardLength12b
    << "height_is_present"                << toBson(msg.height_is_present)                // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::PosConfidenceEllipse& msg) {
  return document{}
    << "semi_major_confidence"  << toBson(msg.semi_major_confidence)  // etsi_its_cpm_ts_msgs/SemiAxisLength
    << "semi_minor_confidence"  << toBson(msg.semi_minor_confidence)  // etsi_its_cpm_ts_msgs/SemiAxisLength
    << "semi_major_orientation" << toBson(msg.semi_major_orientation) // etsi_its_cpm_ts_msgs/HeadingValue
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShape& msg) {
  return document{}
    << "shape_reference_point"                     << toBson(msg.shape_reference_point)                     // etsi_its_cpm_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present"          << toBson(msg.shape_reference_point_is_present)          // bool
    << "range"                                     << toBson(msg.range)                                     // etsi_its_cpm_ts_msgs/StandardLength12b
    << "stationary_horizontal_opening_angle_start" << toBson(msg.stationary_horizontal_opening_angle_start) // etsi_its_cpm_ts_msgs/Wgs84AngleValue
    << "stationary_horizontal_opening_angle_end"   << toBson(msg.stationary_horizontal_opening_angle_end)   // etsi_its_cpm_ts_msgs/Wgs84AngleValue
    << "vertical_opening_angle_start"              << toBson(msg.vertical_opening_angle_start)              // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_start_is_present"   << toBson(msg.vertical_opening_angle_start_is_present)   // bool
    << "vertical_opening_angle_end"                << toBson(msg.vertical_opening_angle_end)                // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_end_is_present"     << toBson(msg.vertical_opening_angle_end_is_present)     // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShapeDetails& msg) {
  return document{}
    << "range"                                   << toBson(msg.range)                                   // etsi_its_cpm_ts_msgs/StandardLength12b
    << "horizontal_opening_angle_start"          << toBson(msg.horizontal_opening_angle_start)          // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "horizontal_opening_angle_end"            << toBson(msg.horizontal_opening_angle_end)            // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_start"            << toBson(msg.vertical_opening_angle_start)            // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_start_is_present" << toBson(msg.vertical_opening_angle_start_is_present) // bool
    << "vertical_opening_angle_end"              << toBson(msg.vertical_opening_angle_end)              // etsi_its_cpm_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_end_is_present"   << toBson(msg.vertical_opening_angle_end_is_present)   // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShapes& msg) {
  return document{}
    << "ref_point_id"            << toBson(msg.ref_point_id)            // etsi_its_cpm_ts_msgs/Identifier1B
    << "x_coordinate"            << toBson(msg.x_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinateSmall
    << "y_coordinate"            << toBson(msg.y_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinateSmall
    << "z_coordinate"            << toBson(msg.z_coordinate)            // etsi_its_cpm_ts_msgs/CartesianCoordinateSmall
    << "z_coordinate_is_present" << toBson(msg.z_coordinate_is_present) // bool
    << "radial_shapes_list"      << toBson(msg.radial_shapes_list)      // etsi_its_cpm_ts_msgs/RadialShapesList
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::RadialShapesList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/RadialShapeDetails[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::RectangularShape& msg) {
  return document{}
    << "center_point"            << toBson(msg.center_point)            // etsi_its_cpm_ts_msgs/CartesianPosition3d
    << "center_point_is_present" << toBson(msg.center_point_is_present) // bool
    << "semi_length"             << toBson(msg.semi_length)             // etsi_its_cpm_ts_msgs/StandardLength12b
    << "semi_breadth"            << toBson(msg.semi_breadth)            // etsi_its_cpm_ts_msgs/StandardLength12b
    << "orientation"             << toBson(msg.orientation)             // etsi_its_cpm_ts_msgs/Wgs84AngleValue
    << "orientation_is_present"  << toBson(msg.orientation_is_present)  // bool
    << "height"                  << toBson(msg.height)                  // etsi_its_cpm_ts_msgs/StandardLength12b
    << "height_is_present"       << toBson(msg.height_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::ReferencePosition& msg) {
  return document{}
    << "latitude"                    << toBson(msg.latitude)                    // etsi_its_cpm_ts_msgs/Latitude
    << "longitude"                   << toBson(msg.longitude)                   // etsi_its_cpm_ts_msgs/Longitude
    << "position_confidence_ellipse" << toBson(msg.position_confidence_ellipse) // etsi_its_cpm_ts_msgs/PosConfidenceEllipse
    << "altitude"                    << toBson(msg.altitude)                    // etsi_its_cpm_ts_msgs/Altitude
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::RoadSegmentReferenceId& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_cpm_ts_msgs/Identifier2B
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_cpm_ts_msgs/Identifier2B
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SemiAxisLength& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SensorInformation& msg) {
  return document{}
    << "sensor_id"                               << toBson(msg.sensor_id)                               // etsi_its_cpm_ts_msgs/Identifier1B
    << "sensor_type"                             << toBson(msg.sensor_type)                             // etsi_its_cpm_ts_msgs/SensorType
    << "perception_region_shape"                 << toBson(msg.perception_region_shape)                 // etsi_its_cpm_ts_msgs/Shape
    << "perception_region_shape_is_present"      << toBson(msg.perception_region_shape_is_present)      // bool
    << "perception_region_confidence"            << toBson(msg.perception_region_confidence)            // etsi_its_cpm_ts_msgs/ConfidenceLevel
    << "perception_region_confidence_is_present" << toBson(msg.perception_region_confidence_is_present) // bool
    << "shadowing_applies"                       << toBson(msg.shadowing_applies)                       // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SensorInformationContainer& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/SensorInformation[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SensorType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SequenceOfCartesianPosition3d& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/CartesianPosition3d[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SequenceOfIdentifier1B& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/Identifier1B[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Shape& msg) {
  return document{}
    << "choice"        << toBson(msg.choice)        // uint8
    << "rectangular"   << toBson(msg.rectangular)   // etsi_its_cpm_ts_msgs/RectangularShape
    << "circular"      << toBson(msg.circular)      // etsi_its_cpm_ts_msgs/CircularShape
    << "polygonal"     << toBson(msg.polygonal)     // etsi_its_cpm_ts_msgs/PolygonalShape
    << "elliptical"    << toBson(msg.elliptical)    // etsi_its_cpm_ts_msgs/EllipticalShape
    << "radial"        << toBson(msg.radial)        // etsi_its_cpm_ts_msgs/RadialShape
    << "radial_shapes" << toBson(msg.radial_shapes) // etsi_its_cpm_ts_msgs/RadialShapes
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Speed& msg) {
  return document{}
    << "speed_value"      << toBson(msg.speed_value)      // etsi_its_cpm_ts_msgs/SpeedValue
    << "speed_confidence" << toBson(msg.speed_confidence) // etsi_its_cpm_ts_msgs/SpeedConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SpeedConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::SpeedValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::StandardLength12b& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::StandardLength1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::StationId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::TimestampIts& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint64
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::TrafficParticipantType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::TrailerData& msg) {
  return document{}
    << "ref_point_id"              << toBson(msg.ref_point_id)              // etsi_its_cpm_ts_msgs/Identifier1B
    << "hitch_point_offset"        << toBson(msg.hitch_point_offset)        // etsi_its_cpm_ts_msgs/StandardLength1B
    << "front_overhang"            << toBson(msg.front_overhang)            // etsi_its_cpm_ts_msgs/StandardLength1B
    << "front_overhang_is_present" << toBson(msg.front_overhang_is_present) // bool
    << "rear_overhang"             << toBson(msg.rear_overhang)             // etsi_its_cpm_ts_msgs/StandardLength1B
    << "rear_overhang_is_present"  << toBson(msg.rear_overhang_is_present)  // bool
    << "trailer_width"             << toBson(msg.trailer_width)             // etsi_its_cpm_ts_msgs/VehicleWidth
    << "trailer_width_is_present"  << toBson(msg.trailer_width_is_present)  // bool
    << "hitch_angle"               << toBson(msg.hitch_angle)               // etsi_its_cpm_ts_msgs/CartesianAngle
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::TrailerDataSet& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/TrailerData[]
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VehicleWidth& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Velocity3dWithConfidence& msg) {
  return document{}
    << "choice"             << toBson(msg.choice)             // uint8
    << "polar_velocity"     << toBson(msg.polar_velocity)     // etsi_its_cpm_ts_msgs/VelocityPolarWithZ
    << "cartesian_velocity" << toBson(msg.cartesian_velocity) // etsi_its_cpm_ts_msgs/VelocityCartesian
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityCartesian& msg) {
  return document{}
    << "x_velocity"            << toBson(msg.x_velocity)            // etsi_its_cpm_ts_msgs/VelocityComponent
    << "y_velocity"            << toBson(msg.y_velocity)            // etsi_its_cpm_ts_msgs/VelocityComponent
    << "z_velocity"            << toBson(msg.z_velocity)            // etsi_its_cpm_ts_msgs/VelocityComponent
    << "z_velocity_is_present" << toBson(msg.z_velocity_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityComponent& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/VelocityComponentValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/SpeedConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityComponentValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VelocityPolarWithZ& msg) {
  return document{}
    << "velocity_magnitude"    << toBson(msg.velocity_magnitude)    // etsi_its_cpm_ts_msgs/Speed
    << "velocity_direction"    << toBson(msg.velocity_direction)    // etsi_its_cpm_ts_msgs/CartesianAngle
    << "z_velocity"            << toBson(msg.z_velocity)            // etsi_its_cpm_ts_msgs/VelocityComponent
    << "z_velocity_is_present" << toBson(msg.z_velocity_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruClusterInformation& msg) {
  return document{}
    << "cluster_id"                            << toBson(msg.cluster_id)                            // etsi_its_cpm_ts_msgs/Identifier1B
    << "cluster_id_is_present"                 << toBson(msg.cluster_id_is_present)                 // bool
    << "cluster_bounding_box_shape"            << toBson(msg.cluster_bounding_box_shape)            // etsi_its_cpm_ts_msgs/Shape
    << "cluster_bounding_box_shape_is_present" << toBson(msg.cluster_bounding_box_shape_is_present) // bool
    << "cluster_cardinality_size"              << toBson(msg.cluster_cardinality_size)              // etsi_its_cpm_ts_msgs/CardinalNumber1B
    << "cluster_profiles"                      << toBson(msg.cluster_profiles)                      // etsi_its_cpm_ts_msgs/VruClusterProfiles
    << "cluster_profiles_is_present"           << toBson(msg.cluster_profiles_is_present)           // bool
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruClusterProfiles& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruProfileAndSubprofile& msg) {
  return document{}
    << "choice"                          << toBson(msg.choice)                          // uint8
    << "pedestrian"                      << toBson(msg.pedestrian)                      // etsi_its_cpm_ts_msgs/VruSubProfilePedestrian
    << "bicyclist_and_light_vru_vehicle" << toBson(msg.bicyclist_and_light_vru_vehicle) // etsi_its_cpm_ts_msgs/VruSubProfileBicyclist
    << "motorcyclist"                    << toBson(msg.motorcyclist)                    // etsi_its_cpm_ts_msgs/VruSubProfileMotorcyclist
    << "animal"                          << toBson(msg.animal)                          // etsi_its_cpm_ts_msgs/VruSubProfileAnimal
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfileAnimal& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfileBicyclist& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfileMotorcyclist& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::VruSubProfilePedestrian& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Wgs84Angle& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_cpm_ts_msgs/Wgs84AngleValue
    << "confidence" << toBson(msg.confidence) // etsi_its_cpm_ts_msgs/Wgs84AngleConfidence
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Wgs84AngleConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::Wgs84AngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::WrappedCpmContainer& msg) {
  return document{}
    << "container_id"                                 << toBson(msg.container_id)                                 // etsi_its_cpm_ts_msgs/CpmContainerId
    << "container_data_originating_vehicle_container" << toBson(msg.container_data_originating_vehicle_container) // etsi_its_cpm_ts_msgs/OriginatingVehicleContainer
    << "container_data_originating_rsu_container"     << toBson(msg.container_data_originating_rsu_container)     // etsi_its_cpm_ts_msgs/OriginatingRsuContainer
    << "container_data_sensor_information_container"  << toBson(msg.container_data_sensor_information_container)  // etsi_its_cpm_ts_msgs/SensorInformationContainer
    << "container_data_perception_region_container"   << toBson(msg.container_data_perception_region_container)   // etsi_its_cpm_ts_msgs/PerceptionRegionContainer
    << "container_data_perceived_object_container"    << toBson(msg.container_data_perceived_object_container)    // etsi_its_cpm_ts_msgs/PerceivedObjectContainer
  << finalize;
}

value toBson(const etsi_its_cpm_ts_msgs::msg::WrappedCpmContainers& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_cpm_ts_msgs/WrappedCpmContainer[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ActionID& msg) {
  return document{}
    << "originating_station_id" << toBson(msg.originating_station_id) // etsi_its_denm_msgs/StationID
    << "sequence_number"        << toBson(msg.sequence_number)        // etsi_its_denm_msgs/SequenceNumber
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::AlacarteContainer& msg) {
  return document{}
    << "lane_position"                   << toBson(msg.lane_position)                   // etsi_its_denm_msgs/LanePosition
    << "lane_position_is_present"        << toBson(msg.lane_position_is_present)        // bool
    << "impact_reduction"                << toBson(msg.impact_reduction)                // etsi_its_denm_msgs/ImpactReductionContainer
    << "impact_reduction_is_present"     << toBson(msg.impact_reduction_is_present)     // bool
    << "external_temperature"            << toBson(msg.external_temperature)            // etsi_its_denm_msgs/Temperature
    << "external_temperature_is_present" << toBson(msg.external_temperature_is_present) // bool
    << "road_works"                      << toBson(msg.road_works)                      // etsi_its_denm_msgs/RoadWorksContainerExtended
    << "road_works_is_present"           << toBson(msg.road_works_is_present)           // bool
    << "positioning_solution"            << toBson(msg.positioning_solution)            // etsi_its_denm_msgs/PositioningSolutionType
    << "positioning_solution_is_present" << toBson(msg.positioning_solution_is_present) // bool
    << "stationary_vehicle"              << toBson(msg.stationary_vehicle)              // etsi_its_denm_msgs/StationaryVehicleContainer
    << "stationary_vehicle_is_present"   << toBson(msg.stationary_vehicle_is_present)   // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Altitude& msg) {
  return document{}
    << "altitude_value"      << toBson(msg.altitude_value)      // etsi_its_denm_msgs/AltitudeValue
    << "altitude_confidence" << toBson(msg.altitude_confidence) // etsi_its_denm_msgs/AltitudeConfidence
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::AltitudeConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::AltitudeValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::CauseCode& msg) {
  return document{}
    << "cause_code"     << toBson(msg.cause_code)     // etsi_its_denm_msgs/CauseCodeType
    << "sub_cause_code" << toBson(msg.sub_cause_code) // etsi_its_denm_msgs/SubCauseCodeType
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::CauseCodeType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ClosedLanes& msg) {
  return document{}
    << "innerhard_shoulder_status"            << toBson(msg.innerhard_shoulder_status)            // etsi_its_denm_msgs/HardShoulderStatus
    << "innerhard_shoulder_status_is_present" << toBson(msg.innerhard_shoulder_status_is_present) // bool
    << "outerhard_shoulder_status"            << toBson(msg.outerhard_shoulder_status)            // etsi_its_denm_msgs/HardShoulderStatus
    << "outerhard_shoulder_status_is_present" << toBson(msg.outerhard_shoulder_status_is_present) // bool
    << "driving_lane_status"                  << toBson(msg.driving_lane_status)                  // etsi_its_denm_msgs/DrivingLaneStatus
    << "driving_lane_status_is_present"       << toBson(msg.driving_lane_status_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DENM& msg) {
  return document{}
    << "header" << toBson(msg.header) // etsi_its_denm_msgs/ItsPduHeader
    << "denm"   << toBson(msg.denm)   // etsi_its_denm_msgs/DecentralizedEnvironmentalNotificationMessage
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DangerousGoodsBasic& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DangerousGoodsExtended& msg) {
  return document{}
    << "dangerous_goods_type"             << toBson(msg.dangerous_goods_type)             // etsi_its_denm_msgs/DangerousGoodsBasic
    << "un_number"                        << toBson(msg.un_number)                        // uint16
    << "elevated_temperature"             << toBson(msg.elevated_temperature)             // bool
    << "tunnels_restricted"               << toBson(msg.tunnels_restricted)               // bool
    << "limited_quantity"                 << toBson(msg.limited_quantity)                 // bool
    << "emergency_action_code"            << toBson(msg.emergency_action_code)            // string
    << "emergency_action_code_is_present" << toBson(msg.emergency_action_code_is_present) // bool
    << "phone_number"                     << toBson(msg.phone_number)                     // etsi_its_denm_msgs/PhoneNumber
    << "phone_number_is_present"          << toBson(msg.phone_number_is_present)          // bool
    << "company_name"                     << toBson(msg.company_name)                     // string
    << "company_name_is_present"          << toBson(msg.company_name_is_present)          // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DecentralizedEnvironmentalNotificationMessage& msg) {
  return document{}
    << "management"           << toBson(msg.management)           // etsi_its_denm_msgs/ManagementContainer
    << "situation"            << toBson(msg.situation)            // etsi_its_denm_msgs/SituationContainer
    << "situation_is_present" << toBson(msg.situation_is_present) // bool
    << "location"             << toBson(msg.location)             // etsi_its_denm_msgs/LocationContainer
    << "location_is_present"  << toBson(msg.location_is_present)  // bool
    << "alacarte"             << toBson(msg.alacarte)             // etsi_its_denm_msgs/AlacarteContainer
    << "alacarte_is_present"  << toBson(msg.alacarte_is_present)  // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DeltaAltitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DeltaLatitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DeltaLongitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DeltaReferencePosition& msg) {
  return document{}
    << "delta_latitude"  << toBson(msg.delta_latitude)  // etsi_its_denm_msgs/DeltaLatitude
    << "delta_longitude" << toBson(msg.delta_longitude) // etsi_its_denm_msgs/DeltaLongitude
    << "delta_altitude"  << toBson(msg.delta_altitude)  // etsi_its_denm_msgs/DeltaAltitude
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::DrivingLaneStatus& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::EnergyStorageType& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::EventHistory& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/EventPoint[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::EventPoint& msg) {
  return document{}
    << "event_position"              << toBson(msg.event_position)              // etsi_its_denm_msgs/DeltaReferencePosition
    << "event_delta_time"            << toBson(msg.event_delta_time)            // etsi_its_denm_msgs/PathDeltaTime
    << "event_delta_time_is_present" << toBson(msg.event_delta_time_is_present) // bool
    << "information_quality"         << toBson(msg.information_quality)         // etsi_its_denm_msgs/InformationQuality
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::HardShoulderStatus& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Heading& msg) {
  return document{}
    << "heading_value"      << toBson(msg.heading_value)      // etsi_its_denm_msgs/HeadingValue
    << "heading_confidence" << toBson(msg.heading_confidence) // etsi_its_denm_msgs/HeadingConfidence
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::HeadingConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::HeadingValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::HeightLonCarr& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ImpactReductionContainer& msg) {
  return document{}
    << "height_lon_carr_left"        << toBson(msg.height_lon_carr_left)        // etsi_its_denm_msgs/HeightLonCarr
    << "height_lon_carr_right"       << toBson(msg.height_lon_carr_right)       // etsi_its_denm_msgs/HeightLonCarr
    << "pos_lon_carr_left"           << toBson(msg.pos_lon_carr_left)           // etsi_its_denm_msgs/PosLonCarr
    << "pos_lon_carr_right"          << toBson(msg.pos_lon_carr_right)          // etsi_its_denm_msgs/PosLonCarr
    << "position_of_pillars"         << toBson(msg.position_of_pillars)         // etsi_its_denm_msgs/PositionOfPillars
    << "pos_cent_mass"               << toBson(msg.pos_cent_mass)               // etsi_its_denm_msgs/PosCentMass
    << "wheel_base_vehicle"          << toBson(msg.wheel_base_vehicle)          // etsi_its_denm_msgs/WheelBaseVehicle
    << "turning_radius"              << toBson(msg.turning_radius)              // etsi_its_denm_msgs/TurningRadius
    << "pos_front_ax"                << toBson(msg.pos_front_ax)                // etsi_its_denm_msgs/PosFrontAx
    << "position_of_occupants"       << toBson(msg.position_of_occupants)       // etsi_its_denm_msgs/PositionOfOccupants
    << "vehicle_mass"                << toBson(msg.vehicle_mass)                // etsi_its_denm_msgs/VehicleMass
    << "request_response_indication" << toBson(msg.request_response_indication) // etsi_its_denm_msgs/RequestResponseIndication
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::InformationQuality& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ItineraryPath& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/ReferencePosition[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // uint8
    << "message_id"       << toBson(msg.message_id)       // uint8
    << "station_id"       << toBson(msg.station_id)       // etsi_its_denm_msgs/StationID
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::LanePosition& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Latitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::LightBarSirenInUse& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::LocationContainer& msg) {
  return document{}
    << "event_speed"                       << toBson(msg.event_speed)                       // etsi_its_denm_msgs/Speed
    << "event_speed_is_present"            << toBson(msg.event_speed_is_present)            // bool
    << "event_position_heading"            << toBson(msg.event_position_heading)            // etsi_its_denm_msgs/Heading
    << "event_position_heading_is_present" << toBson(msg.event_position_heading_is_present) // bool
    << "traces"                            << toBson(msg.traces)                            // etsi_its_denm_msgs/Traces
    << "road_type"                         << toBson(msg.road_type)                         // etsi_its_denm_msgs/RoadType
    << "road_type_is_present"              << toBson(msg.road_type_is_present)              // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Longitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ManagementContainer& msg) {
  return document{}
    << "action_id"                              << toBson(msg.action_id)                              // etsi_its_denm_msgs/ActionID
    << "detection_time"                         << toBson(msg.detection_time)                         // etsi_its_denm_msgs/TimestampIts
    << "reference_time"                         << toBson(msg.reference_time)                         // etsi_its_denm_msgs/TimestampIts
    << "termination"                            << toBson(msg.termination)                            // etsi_its_denm_msgs/Termination
    << "termination_is_present"                 << toBson(msg.termination_is_present)                 // bool
    << "event_position"                         << toBson(msg.event_position)                         // etsi_its_denm_msgs/ReferencePosition
    << "relevance_distance"                     << toBson(msg.relevance_distance)                     // etsi_its_denm_msgs/RelevanceDistance
    << "relevance_distance_is_present"          << toBson(msg.relevance_distance_is_present)          // bool
    << "relevance_traffic_direction"            << toBson(msg.relevance_traffic_direction)            // etsi_its_denm_msgs/RelevanceTrafficDirection
    << "relevance_traffic_direction_is_present" << toBson(msg.relevance_traffic_direction_is_present) // bool
    << "validity_duration"                      << toBson(msg.validity_duration)                      // etsi_its_denm_msgs/ValidityDuration
    << "transmission_interval"                  << toBson(msg.transmission_interval)                  // etsi_its_denm_msgs/TransmissionInterval
    << "transmission_interval_is_present"       << toBson(msg.transmission_interval_is_present)       // bool
    << "station_type"                           << toBson(msg.station_type)                           // etsi_its_denm_msgs/StationType
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::NumberOfOccupants& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PathDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PathHistory& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/PathPoint[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PathPoint& msg) {
  return document{}
    << "path_position"              << toBson(msg.path_position)              // etsi_its_denm_msgs/DeltaReferencePosition
    << "path_delta_time"            << toBson(msg.path_delta_time)            // etsi_its_denm_msgs/PathDeltaTime
    << "path_delta_time_is_present" << toBson(msg.path_delta_time_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PhoneNumber& msg) {
  return document{}
    << "value" << toBson(msg.value) // string
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PosCentMass& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PosConfidenceEllipse& msg) {
  return document{}
    << "semi_major_confidence"  << toBson(msg.semi_major_confidence)  // etsi_its_denm_msgs/SemiAxisLength
    << "semi_minor_confidence"  << toBson(msg.semi_minor_confidence)  // etsi_its_denm_msgs/SemiAxisLength
    << "semi_major_orientation" << toBson(msg.semi_major_orientation) // etsi_its_denm_msgs/HeadingValue
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PosFrontAx& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PosLonCarr& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PosPillar& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PositionOfOccupants& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PositionOfPillars& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/PosPillar[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::PositioningSolutionType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ReferenceDenms& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/ActionID[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ReferencePosition& msg) {
  return document{}
    << "latitude"                    << toBson(msg.latitude)                    // etsi_its_denm_msgs/Latitude
    << "longitude"                   << toBson(msg.longitude)                   // etsi_its_denm_msgs/Longitude
    << "position_confidence_ellipse" << toBson(msg.position_confidence_ellipse) // etsi_its_denm_msgs/PosConfidenceEllipse
    << "altitude"                    << toBson(msg.altitude)                    // etsi_its_denm_msgs/Altitude
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::RelevanceDistance& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::RelevanceTrafficDirection& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::RequestResponseIndication& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::RestrictedTypes& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/StationType[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::RoadType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::RoadWorksContainerExtended& msg) {
  return document{}
    << "light_bar_siren_in_use"                << toBson(msg.light_bar_siren_in_use)                // etsi_its_denm_msgs/LightBarSirenInUse
    << "light_bar_siren_in_use_is_present"     << toBson(msg.light_bar_siren_in_use_is_present)     // bool
    << "closed_lanes"                          << toBson(msg.closed_lanes)                          // etsi_its_denm_msgs/ClosedLanes
    << "closed_lanes_is_present"               << toBson(msg.closed_lanes_is_present)               // bool
    << "restriction"                           << toBson(msg.restriction)                           // etsi_its_denm_msgs/RestrictedTypes
    << "restriction_is_present"                << toBson(msg.restriction_is_present)                // bool
    << "speed_limit"                           << toBson(msg.speed_limit)                           // etsi_its_denm_msgs/SpeedLimit
    << "speed_limit_is_present"                << toBson(msg.speed_limit_is_present)                // bool
    << "incident_indication"                   << toBson(msg.incident_indication)                   // etsi_its_denm_msgs/CauseCode
    << "incident_indication_is_present"        << toBson(msg.incident_indication_is_present)        // bool
    << "recommended_path"                      << toBson(msg.recommended_path)                      // etsi_its_denm_msgs/ItineraryPath
    << "recommended_path_is_present"           << toBson(msg.recommended_path_is_present)           // bool
    << "starting_point_speed_limit"            << toBson(msg.starting_point_speed_limit)            // etsi_its_denm_msgs/DeltaReferencePosition
    << "starting_point_speed_limit_is_present" << toBson(msg.starting_point_speed_limit_is_present) // bool
    << "traffic_flow_rule"                     << toBson(msg.traffic_flow_rule)                     // etsi_its_denm_msgs/TrafficRule
    << "traffic_flow_rule_is_present"          << toBson(msg.traffic_flow_rule_is_present)          // bool
    << "reference_denms"                       << toBson(msg.reference_denms)                       // etsi_its_denm_msgs/ReferenceDenms
    << "reference_denms_is_present"            << toBson(msg.reference_denms_is_present)            // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SemiAxisLength& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SequenceNumber& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SituationContainer& msg) {
  return document{}
    << "information_quality"      << toBson(msg.information_quality)      // etsi_its_denm_msgs/InformationQuality
    << "event_type"               << toBson(msg.event_type)               // etsi_its_denm_msgs/CauseCode
    << "linked_cause"             << toBson(msg.linked_cause)             // etsi_its_denm_msgs/CauseCode
    << "linked_cause_is_present"  << toBson(msg.linked_cause_is_present)  // bool
    << "event_history"            << toBson(msg.event_history)            // etsi_its_denm_msgs/EventHistory
    << "event_history_is_present" << toBson(msg.event_history_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Speed& msg) {
  return document{}
    << "speed_value"      << toBson(msg.speed_value)      // etsi_its_denm_msgs/SpeedValue
    << "speed_confidence" << toBson(msg.speed_confidence) // etsi_its_denm_msgs/SpeedConfidence
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SpeedConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SpeedLimit& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SpeedValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::StationID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::StationType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::StationarySince& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::StationaryVehicleContainer& msg) {
  return document{}
    << "stationary_since"                    << toBson(msg.stationary_since)                    // etsi_its_denm_msgs/StationarySince
    << "stationary_since_is_present"         << toBson(msg.stationary_since_is_present)         // bool
    << "stationary_cause"                    << toBson(msg.stationary_cause)                    // etsi_its_denm_msgs/CauseCode
    << "stationary_cause_is_present"         << toBson(msg.stationary_cause_is_present)         // bool
    << "carrying_dangerous_goods"            << toBson(msg.carrying_dangerous_goods)            // etsi_its_denm_msgs/DangerousGoodsExtended
    << "carrying_dangerous_goods_is_present" << toBson(msg.carrying_dangerous_goods_is_present) // bool
    << "number_of_occupants"                 << toBson(msg.number_of_occupants)                 // etsi_its_denm_msgs/NumberOfOccupants
    << "number_of_occupants_is_present"      << toBson(msg.number_of_occupants_is_present)      // bool
    << "vehicle_identification"              << toBson(msg.vehicle_identification)              // etsi_its_denm_msgs/VehicleIdentification
    << "vehicle_identification_is_present"   << toBson(msg.vehicle_identification_is_present)   // bool
    << "energy_storage_type"                 << toBson(msg.energy_storage_type)                 // etsi_its_denm_msgs/EnergyStorageType
    << "energy_storage_type_is_present"      << toBson(msg.energy_storage_type_is_present)      // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::SubCauseCodeType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Temperature& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Termination& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::TimestampIts& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint64
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::Traces& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_denm_msgs/PathHistory[]
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::TrafficRule& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::TransmissionInterval& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::TurningRadius& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::VDS& msg) {
  return document{}
    << "value" << toBson(msg.value) // string
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::ValidityDuration& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::VehicleIdentification& msg) {
  return document{}
    << "w_m_inumber"            << toBson(msg.w_m_inumber)            // etsi_its_denm_msgs/WMInumber
    << "w_m_inumber_is_present" << toBson(msg.w_m_inumber_is_present) // bool
    << "v_ds"                   << toBson(msg.v_ds)                   // etsi_its_denm_msgs/VDS
    << "v_ds_is_present"        << toBson(msg.v_ds_is_present)        // bool
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::VehicleMass& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::WMInumber& msg) {
  return document{}
    << "value" << toBson(msg.value) // string
  << finalize;
}

value toBson(const etsi_its_denm_msgs::msg::WheelBaseVehicle& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::AllowedManeuvers& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Angle& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::ApproachID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::ComputedLane& msg) {
  return document{}
    << "reference_lane_id"      << toBson(msg.reference_lane_id)      // etsi_its_mapem_ts_msgs/LaneID
    << "offset_xaxis_choice"    << toBson(msg.offset_xaxis_choice)    // uint8
    << "offset_xaxis_small"     << toBson(msg.offset_xaxis_small)     // etsi_its_mapem_ts_msgs/DrivenLineOffsetSm
    << "offset_xaxis_large"     << toBson(msg.offset_xaxis_large)     // etsi_its_mapem_ts_msgs/DrivenLineOffsetLg
    << "offset_yaxis_choice"    << toBson(msg.offset_yaxis_choice)    // uint8
    << "offset_yaxis_small"     << toBson(msg.offset_yaxis_small)     // etsi_its_mapem_ts_msgs/DrivenLineOffsetSm
    << "offset_yaxis_large"     << toBson(msg.offset_yaxis_large)     // etsi_its_mapem_ts_msgs/DrivenLineOffsetLg
    << "rotate_xy"              << toBson(msg.rotate_xy)              // etsi_its_mapem_ts_msgs/Angle
    << "rotate_xy_is_present"   << toBson(msg.rotate_xy_is_present)   // bool
    << "scale_xaxis"            << toBson(msg.scale_xaxis)            // etsi_its_mapem_ts_msgs/ScaleB12
    << "scale_xaxis_is_present" << toBson(msg.scale_xaxis_is_present) // bool
    << "scale_yaxis"            << toBson(msg.scale_yaxis)            // etsi_its_mapem_ts_msgs/ScaleB12
    << "scale_yaxis_is_present" << toBson(msg.scale_yaxis_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::ConnectingLane& msg) {
  return document{}
    << "lane"                << toBson(msg.lane)                // etsi_its_mapem_ts_msgs/LaneID
    << "maneuver"            << toBson(msg.maneuver)            // etsi_its_mapem_ts_msgs/AllowedManeuvers
    << "maneuver_is_present" << toBson(msg.maneuver_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Connection& msg) {
  return document{}
    << "connecting_lane"                << toBson(msg.connecting_lane)                // etsi_its_mapem_ts_msgs/ConnectingLane
    << "remote_intersection"            << toBson(msg.remote_intersection)            // etsi_its_mapem_ts_msgs/IntersectionReferenceID
    << "remote_intersection_is_present" << toBson(msg.remote_intersection_is_present) // bool
    << "signal_group"                   << toBson(msg.signal_group)                   // etsi_its_mapem_ts_msgs/SignalGroupID
    << "signal_group_is_present"        << toBson(msg.signal_group_is_present)        // bool
    << "user_class"                     << toBson(msg.user_class)                     // etsi_its_mapem_ts_msgs/RestrictionClassID
    << "user_class_is_present"          << toBson(msg.user_class_is_present)          // bool
    << "connection_id"                  << toBson(msg.connection_id)                  // etsi_its_mapem_ts_msgs/LaneConnectionID
    << "connection_id_is_present"       << toBson(msg.connection_id_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::ConnectsToList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/Connection[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::DataParameters& msg) {
  return document{}
    << "process_method"               << toBson(msg.process_method)               // string
    << "process_method_is_present"    << toBson(msg.process_method_is_present)    // bool
    << "process_agency"               << toBson(msg.process_agency)               // string
    << "process_agency_is_present"    << toBson(msg.process_agency_is_present)    // bool
    << "last_checked_date"            << toBson(msg.last_checked_date)            // string
    << "last_checked_date_is_present" << toBson(msg.last_checked_date_is_present) // bool
    << "geoid_used"                   << toBson(msg.geoid_used)                   // string
    << "geoid_used_is_present"        << toBson(msg.geoid_used_is_present)        // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::DeltaAngle& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::DescriptiveName& msg) {
  return document{}
    << "value" << toBson(msg.value) // string
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::DrivenLineOffsetLg& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::DrivenLineOffsetSm& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Elevation& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::GenericLane& msg) {
  return document{}
    << "lane_id"                     << toBson(msg.lane_id)                     // etsi_its_mapem_ts_msgs/LaneID
    << "name"                        << toBson(msg.name)                        // etsi_its_mapem_ts_msgs/DescriptiveName
    << "name_is_present"             << toBson(msg.name_is_present)             // bool
    << "ingress_approach"            << toBson(msg.ingress_approach)            // etsi_its_mapem_ts_msgs/ApproachID
    << "ingress_approach_is_present" << toBson(msg.ingress_approach_is_present) // bool
    << "egress_approach"             << toBson(msg.egress_approach)             // etsi_its_mapem_ts_msgs/ApproachID
    << "egress_approach_is_present"  << toBson(msg.egress_approach_is_present)  // bool
    << "lane_attributes"             << toBson(msg.lane_attributes)             // etsi_its_mapem_ts_msgs/LaneAttributes
    << "maneuvers"                   << toBson(msg.maneuvers)                   // etsi_its_mapem_ts_msgs/AllowedManeuvers
    << "maneuvers_is_present"        << toBson(msg.maneuvers_is_present)        // bool
    << "node_list"                   << toBson(msg.node_list)                   // etsi_its_mapem_ts_msgs/NodeListXY
    << "connects_to"                 << toBson(msg.connects_to)                 // etsi_its_mapem_ts_msgs/ConnectsToList
    << "connects_to_is_present"      << toBson(msg.connects_to_is_present)      // bool
    << "overlays"                    << toBson(msg.overlays)                    // etsi_its_mapem_ts_msgs/OverlayLaneList
    << "overlays_is_present"         << toBson(msg.overlays_is_present)         // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionGeometry& msg) {
  return document{}
    << "name"                             << toBson(msg.name)                             // etsi_its_mapem_ts_msgs/DescriptiveName
    << "name_is_present"                  << toBson(msg.name_is_present)                  // bool
    << "id"                               << toBson(msg.id)                               // etsi_its_mapem_ts_msgs/IntersectionReferenceID
    << "revision"                         << toBson(msg.revision)                         // etsi_its_mapem_ts_msgs/MsgCount
    << "ref_point"                        << toBson(msg.ref_point)                        // etsi_its_mapem_ts_msgs/Position3D
    << "lane_width"                       << toBson(msg.lane_width)                       // etsi_its_mapem_ts_msgs/LaneWidth
    << "lane_width_is_present"            << toBson(msg.lane_width_is_present)            // bool
    << "speed_limits"                     << toBson(msg.speed_limits)                     // etsi_its_mapem_ts_msgs/SpeedLimitList
    << "speed_limits_is_present"          << toBson(msg.speed_limits_is_present)          // bool
    << "lane_set"                         << toBson(msg.lane_set)                         // etsi_its_mapem_ts_msgs/LaneList
    << "preempt_priority_data"            << toBson(msg.preempt_priority_data)            // etsi_its_mapem_ts_msgs/PreemptPriorityList
    << "preempt_priority_data_is_present" << toBson(msg.preempt_priority_data_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionGeometryList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/IntersectionGeometry[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::IntersectionReferenceID& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_mapem_ts_msgs/RoadRegulatorID
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_mapem_ts_msgs/IntersectionID
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // uint8
    << "message_id"       << toBson(msg.message_id)       // uint8
    << "station_id"       << toBson(msg.station_id)       // etsi_its_mapem_ts_msgs/StationID
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributes& msg) {
  return document{}
    << "directional_use" << toBson(msg.directional_use) // etsi_its_mapem_ts_msgs/LaneDirection
    << "shared_with"     << toBson(msg.shared_with)     // etsi_its_mapem_ts_msgs/LaneSharing
    << "lane_type"       << toBson(msg.lane_type)       // etsi_its_mapem_ts_msgs/LaneTypeAttributes
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesBarrier& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesBike& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesCrosswalk& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesParking& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesSidewalk& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesStriping& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesTrackedVehicle& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneAttributesVehicle& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneConnectionID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneDataAttribute& msg) {
  return document{}
    << "choice"                  << toBson(msg.choice)                  // uint8
    << "path_end_point_angle"    << toBson(msg.path_end_point_angle)    // etsi_its_mapem_ts_msgs/DeltaAngle
    << "lane_crown_point_center" << toBson(msg.lane_crown_point_center) // etsi_its_mapem_ts_msgs/RoadwayCrownAngle
    << "lane_crown_point_left"   << toBson(msg.lane_crown_point_left)   // etsi_its_mapem_ts_msgs/RoadwayCrownAngle
    << "lane_crown_point_right"  << toBson(msg.lane_crown_point_right)  // etsi_its_mapem_ts_msgs/RoadwayCrownAngle
    << "lane_angle"              << toBson(msg.lane_angle)              // etsi_its_mapem_ts_msgs/MergeDivergeNodeAngle
    << "speed_limits"            << toBson(msg.speed_limits)            // etsi_its_mapem_ts_msgs/SpeedLimitList
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneDataAttributeList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/LaneDataAttribute[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneDirection& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/GenericLane[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneSharing& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes& msg) {
  return document{}
    << "choice"          << toBson(msg.choice)          // uint8
    << "vehicle"         << toBson(msg.vehicle)         // etsi_its_mapem_ts_msgs/LaneAttributesVehicle
    << "crosswalk"       << toBson(msg.crosswalk)       // etsi_its_mapem_ts_msgs/LaneAttributesCrosswalk
    << "bike_lane"       << toBson(msg.bike_lane)       // etsi_its_mapem_ts_msgs/LaneAttributesBike
    << "sidewalk"        << toBson(msg.sidewalk)        // etsi_its_mapem_ts_msgs/LaneAttributesSidewalk
    << "median"          << toBson(msg.median)          // etsi_its_mapem_ts_msgs/LaneAttributesBarrier
    << "striping"        << toBson(msg.striping)        // etsi_its_mapem_ts_msgs/LaneAttributesStriping
    << "tracked_vehicle" << toBson(msg.tracked_vehicle) // etsi_its_mapem_ts_msgs/LaneAttributesTrackedVehicle
    << "parking"         << toBson(msg.parking)         // etsi_its_mapem_ts_msgs/LaneAttributesParking
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LaneWidth& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Latitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LayerID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::LayerType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Longitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::MAPEM& msg) {
  return document{}
    << "header" << toBson(msg.header) // etsi_its_mapem_ts_msgs/ItsPduHeader
    << "map"    << toBson(msg.map)    // etsi_its_mapem_ts_msgs/MapData
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::MapData& msg) {
  return document{}
    << "time_stamp"                  << toBson(msg.time_stamp)                  // etsi_its_mapem_ts_msgs/MinuteOfTheYear
    << "time_stamp_is_present"       << toBson(msg.time_stamp_is_present)       // bool
    << "msg_issue_revision"          << toBson(msg.msg_issue_revision)          // etsi_its_mapem_ts_msgs/MsgCount
    << "layer_type"                  << toBson(msg.layer_type)                  // etsi_its_mapem_ts_msgs/LayerType
    << "layer_type_is_present"       << toBson(msg.layer_type_is_present)       // bool
    << "layer_id"                    << toBson(msg.layer_id)                    // etsi_its_mapem_ts_msgs/LayerID
    << "layer_id_is_present"         << toBson(msg.layer_id_is_present)         // bool
    << "intersections"               << toBson(msg.intersections)               // etsi_its_mapem_ts_msgs/IntersectionGeometryList
    << "intersections_is_present"    << toBson(msg.intersections_is_present)    // bool
    << "road_segments"               << toBson(msg.road_segments)               // etsi_its_mapem_ts_msgs/RoadSegmentList
    << "road_segments_is_present"    << toBson(msg.road_segments_is_present)    // bool
    << "data_parameters"             << toBson(msg.data_parameters)             // etsi_its_mapem_ts_msgs/DataParameters
    << "data_parameters_is_present"  << toBson(msg.data_parameters_is_present)  // bool
    << "restriction_list"            << toBson(msg.restriction_list)            // etsi_its_mapem_ts_msgs/RestrictionClassList
    << "restriction_list_is_present" << toBson(msg.restriction_list_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::MergeDivergeNodeAngle& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::MinuteOfTheYear& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::MsgCount& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeAttributeSetXY& msg) {
  return document{}
    << "local_node"             << toBson(msg.local_node)             // etsi_its_mapem_ts_msgs/NodeAttributeXYList
    << "local_node_is_present"  << toBson(msg.local_node_is_present)  // bool
    << "disabled"               << toBson(msg.disabled)               // etsi_its_mapem_ts_msgs/SegmentAttributeXYList
    << "disabled_is_present"    << toBson(msg.disabled_is_present)    // bool
    << "enabled"                << toBson(msg.enabled)                // etsi_its_mapem_ts_msgs/SegmentAttributeXYList
    << "enabled_is_present"     << toBson(msg.enabled_is_present)     // bool
    << "data"                   << toBson(msg.data)                   // etsi_its_mapem_ts_msgs/LaneDataAttributeList
    << "data_is_present"        << toBson(msg.data_is_present)        // bool
    << "d_width"                << toBson(msg.d_width)                // etsi_its_mapem_ts_msgs/OffsetB10
    << "d_width_is_present"     << toBson(msg.d_width_is_present)     // bool
    << "d_elevation"            << toBson(msg.d_elevation)            // etsi_its_mapem_ts_msgs/OffsetB10
    << "d_elevation_is_present" << toBson(msg.d_elevation_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeAttributeXY& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeAttributeXYList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/NodeAttributeXY[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeLLmD64b& msg) {
  return document{}
    << "lon" << toBson(msg.lon) // etsi_its_mapem_ts_msgs/Longitude
    << "lat" << toBson(msg.lat) // etsi_its_mapem_ts_msgs/Latitude
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeListXY& msg) {
  return document{}
    << "choice"   << toBson(msg.choice)   // uint8
    << "nodes"    << toBson(msg.nodes)    // etsi_its_mapem_ts_msgs/NodeSetXY
    << "computed" << toBson(msg.computed) // etsi_its_mapem_ts_msgs/ComputedLane
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY& msg) {
  return document{}
    << "choice"       << toBson(msg.choice)       // uint8
    << "node_xy1"     << toBson(msg.node_xy1)     // etsi_its_mapem_ts_msgs/NodeXY20b
    << "node_xy2"     << toBson(msg.node_xy2)     // etsi_its_mapem_ts_msgs/NodeXY22b
    << "node_xy3"     << toBson(msg.node_xy3)     // etsi_its_mapem_ts_msgs/NodeXY24b
    << "node_xy4"     << toBson(msg.node_xy4)     // etsi_its_mapem_ts_msgs/NodeXY26b
    << "node_xy5"     << toBson(msg.node_xy5)     // etsi_its_mapem_ts_msgs/NodeXY28b
    << "node_xy6"     << toBson(msg.node_xy6)     // etsi_its_mapem_ts_msgs/NodeXY32b
    << "node_lat_lon" << toBson(msg.node_lat_lon) // etsi_its_mapem_ts_msgs/NodeLLmD64b
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeSetXY& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/NodeXY[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY& msg) {
  return document{}
    << "delta"                 << toBson(msg.delta)                 // etsi_its_mapem_ts_msgs/NodeOffsetPointXY
    << "attributes"            << toBson(msg.attributes)            // etsi_its_mapem_ts_msgs/NodeAttributeSetXY
    << "attributes_is_present" << toBson(msg.attributes_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY20b& msg) {
  return document{}
    << "x" << toBson(msg.x) // etsi_its_mapem_ts_msgs/OffsetB10
    << "y" << toBson(msg.y) // etsi_its_mapem_ts_msgs/OffsetB10
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY22b& msg) {
  return document{}
    << "x" << toBson(msg.x) // etsi_its_mapem_ts_msgs/OffsetB11
    << "y" << toBson(msg.y) // etsi_its_mapem_ts_msgs/OffsetB11
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY24b& msg) {
  return document{}
    << "x" << toBson(msg.x) // etsi_its_mapem_ts_msgs/OffsetB12
    << "y" << toBson(msg.y) // etsi_its_mapem_ts_msgs/OffsetB12
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY26b& msg) {
  return document{}
    << "x" << toBson(msg.x) // etsi_its_mapem_ts_msgs/OffsetB13
    << "y" << toBson(msg.y) // etsi_its_mapem_ts_msgs/OffsetB13
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY28b& msg) {
  return document{}
    << "x" << toBson(msg.x) // etsi_its_mapem_ts_msgs/OffsetB14
    << "y" << toBson(msg.y) // etsi_its_mapem_ts_msgs/OffsetB14
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::NodeXY32b& msg) {
  return document{}
    << "x" << toBson(msg.x) // etsi_its_mapem_ts_msgs/OffsetB16
    << "y" << toBson(msg.y) // etsi_its_mapem_ts_msgs/OffsetB16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB10& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB11& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB12& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB13& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB14& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OffsetB16& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::OverlayLaneList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/LaneID[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Position3D& msg) {
  return document{}
    << "lat"                  << toBson(msg.lat)                  // etsi_its_mapem_ts_msgs/Latitude
    << "lon"                  << toBson(msg.lon)                  // etsi_its_mapem_ts_msgs/Longitude
    << "elevation"            << toBson(msg.elevation)            // etsi_its_mapem_ts_msgs/Elevation
    << "elevation_is_present" << toBson(msg.elevation_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::PreemptPriorityList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/SignalControlZone[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RegulatorySpeedLimit& msg) {
  return document{}
    << "type"  << toBson(msg.type)  // etsi_its_mapem_ts_msgs/SpeedLimitType
    << "speed" << toBson(msg.speed) // etsi_its_mapem_ts_msgs/Velocity
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionAppliesTo& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionClassAssignment& msg) {
  return document{}
    << "id"    << toBson(msg.id)    // etsi_its_mapem_ts_msgs/RestrictionClassID
    << "users" << toBson(msg.users) // etsi_its_mapem_ts_msgs/RestrictionUserTypeList
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionClassID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionClassList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/RestrictionClassAssignment[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionUserType& msg) {
  return document{}
    << "choice"     << toBson(msg.choice)     // uint8
    << "basic_type" << toBson(msg.basic_type) // etsi_its_mapem_ts_msgs/RestrictionAppliesTo
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RestrictionUserTypeList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/RestrictionUserType[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadLaneSetList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/GenericLane[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadRegulatorID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegment& msg) {
  return document{}
    << "name"                    << toBson(msg.name)                    // etsi_its_mapem_ts_msgs/DescriptiveName
    << "name_is_present"         << toBson(msg.name_is_present)         // bool
    << "id"                      << toBson(msg.id)                      // etsi_its_mapem_ts_msgs/RoadSegmentReferenceID
    << "revision"                << toBson(msg.revision)                // etsi_its_mapem_ts_msgs/MsgCount
    << "ref_point"               << toBson(msg.ref_point)               // etsi_its_mapem_ts_msgs/Position3D
    << "lane_width"              << toBson(msg.lane_width)              // etsi_its_mapem_ts_msgs/LaneWidth
    << "lane_width_is_present"   << toBson(msg.lane_width_is_present)   // bool
    << "speed_limits"            << toBson(msg.speed_limits)            // etsi_its_mapem_ts_msgs/SpeedLimitList
    << "speed_limits_is_present" << toBson(msg.speed_limits_is_present) // bool
    << "road_lane_set"           << toBson(msg.road_lane_set)           // etsi_its_mapem_ts_msgs/RoadLaneSetList
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegmentID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegmentList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/RoadSegment[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadSegmentReferenceID& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_mapem_ts_msgs/RoadRegulatorID
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_mapem_ts_msgs/RoadSegmentID
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::RoadwayCrownAngle& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::ScaleB12& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::SegmentAttributeXY& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::SegmentAttributeXYList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/SegmentAttributeXY[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::SignalControlZone& msg) {
  return document{}
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::SignalGroupID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::SpeedLimitList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_mapem_ts_msgs/RegulatorySpeedLimit[]
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::SpeedLimitType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::StationID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_mapem_ts_msgs::msg::Velocity& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::AdvisorySpeed& msg) {
  return document{}
    << "type"                  << toBson(msg.type)                  // etsi_its_spatem_ts_msgs/AdvisorySpeedType
    << "speed"                 << toBson(msg.speed)                 // etsi_its_spatem_ts_msgs/SpeedAdvice
    << "speed_is_present"      << toBson(msg.speed_is_present)      // bool
    << "confidence"            << toBson(msg.confidence)            // etsi_its_spatem_ts_msgs/SpeedConfidenceDSRC
    << "confidence_is_present" << toBson(msg.confidence_is_present) // bool
    << "distance"              << toBson(msg.distance)              // etsi_its_spatem_ts_msgs/ZoneLength
    << "distance_is_present"   << toBson(msg.distance_is_present)   // bool
    << "cls"                   << toBson(msg.cls)                   // etsi_its_spatem_ts_msgs/RestrictionClassID
    << "cls_is_present"        << toBson(msg.cls_is_present)        // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::AdvisorySpeedList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_spatem_ts_msgs/AdvisorySpeed[]
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::AdvisorySpeedType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::ConnectionManeuverAssist& msg) {
  return document{}
    << "connection_id"                       << toBson(msg.connection_id)                       // etsi_its_spatem_ts_msgs/LaneConnectionID
    << "queue_length"                        << toBson(msg.queue_length)                        // etsi_its_spatem_ts_msgs/ZoneLength
    << "queue_length_is_present"             << toBson(msg.queue_length_is_present)             // bool
    << "available_storage_length"            << toBson(msg.available_storage_length)            // etsi_its_spatem_ts_msgs/ZoneLength
    << "available_storage_length_is_present" << toBson(msg.available_storage_length_is_present) // bool
    << "wait_on_stop"                        << toBson(msg.wait_on_stop)                        // etsi_its_spatem_ts_msgs/WaitOnStopline
    << "wait_on_stop_is_present"             << toBson(msg.wait_on_stop_is_present)             // bool
    << "ped_bicycle_detect"                  << toBson(msg.ped_bicycle_detect)                  // etsi_its_spatem_ts_msgs/PedestrianBicycleDetect
    << "ped_bicycle_detect_is_present"       << toBson(msg.ped_bicycle_detect_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::DSecond& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::DescriptiveName& msg) {
  return document{}
    << "value" << toBson(msg.value) // string
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::EnabledLaneList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_spatem_ts_msgs/LaneID[]
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionReferenceID& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_spatem_ts_msgs/RoadRegulatorID
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_spatem_ts_msgs/IntersectionID
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionState& msg) {
  return document{}
    << "name"                            << toBson(msg.name)                            // etsi_its_spatem_ts_msgs/DescriptiveName
    << "name_is_present"                 << toBson(msg.name_is_present)                 // bool
    << "id"                              << toBson(msg.id)                              // etsi_its_spatem_ts_msgs/IntersectionReferenceID
    << "revision"                        << toBson(msg.revision)                        // etsi_its_spatem_ts_msgs/MsgCount
    << "status"                          << toBson(msg.status)                          // etsi_its_spatem_ts_msgs/IntersectionStatusObject
    << "moy"                             << toBson(msg.moy)                             // etsi_its_spatem_ts_msgs/MinuteOfTheYear
    << "moy_is_present"                  << toBson(msg.moy_is_present)                  // bool
    << "time_stamp"                      << toBson(msg.time_stamp)                      // etsi_its_spatem_ts_msgs/DSecond
    << "time_stamp_is_present"           << toBson(msg.time_stamp_is_present)           // bool
    << "enabled_lanes"                   << toBson(msg.enabled_lanes)                   // etsi_its_spatem_ts_msgs/EnabledLaneList
    << "enabled_lanes_is_present"        << toBson(msg.enabled_lanes_is_present)        // bool
    << "states"                          << toBson(msg.states)                          // etsi_its_spatem_ts_msgs/MovementList
    << "maneuver_assist_list"            << toBson(msg.maneuver_assist_list)            // etsi_its_spatem_ts_msgs/ManeuverAssistList
    << "maneuver_assist_list_is_present" << toBson(msg.maneuver_assist_list_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionStateList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_spatem_ts_msgs/IntersectionState[]
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::IntersectionStatusObject& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // uint8
    << "message_id"       << toBson(msg.message_id)       // uint8
    << "station_id"       << toBson(msg.station_id)       // etsi_its_spatem_ts_msgs/StationID
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::LaneConnectionID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::LaneID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::ManeuverAssistList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_spatem_ts_msgs/ConnectionManeuverAssist[]
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MinuteOfTheYear& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MovementEvent& msg) {
  return document{}
    << "event_state"       << toBson(msg.event_state)       // etsi_its_spatem_ts_msgs/MovementPhaseState
    << "timing"            << toBson(msg.timing)            // etsi_its_spatem_ts_msgs/TimeChangeDetails
    << "timing_is_present" << toBson(msg.timing_is_present) // bool
    << "speeds"            << toBson(msg.speeds)            // etsi_its_spatem_ts_msgs/AdvisorySpeedList
    << "speeds_is_present" << toBson(msg.speeds_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MovementEventList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_spatem_ts_msgs/MovementEvent[]
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MovementList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_spatem_ts_msgs/MovementState[]
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MovementPhaseState& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MovementState& msg) {
  return document{}
    << "movement_name"                   << toBson(msg.movement_name)                   // etsi_its_spatem_ts_msgs/DescriptiveName
    << "movement_name_is_present"        << toBson(msg.movement_name_is_present)        // bool
    << "signal_group"                    << toBson(msg.signal_group)                    // etsi_its_spatem_ts_msgs/SignalGroupID
    << "state_time_speed"                << toBson(msg.state_time_speed)                // etsi_its_spatem_ts_msgs/MovementEventList
    << "maneuver_assist_list"            << toBson(msg.maneuver_assist_list)            // etsi_its_spatem_ts_msgs/ManeuverAssistList
    << "maneuver_assist_list_is_present" << toBson(msg.maneuver_assist_list_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::MsgCount& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::PedestrianBicycleDetect& msg) {
  return document{}
    << "value" << toBson(msg.value) // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::RestrictionClassID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::RoadRegulatorID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::SPAT& msg) {
  return document{}
    << "time_stamp"            << toBson(msg.time_stamp)            // etsi_its_spatem_ts_msgs/MinuteOfTheYear
    << "time_stamp_is_present" << toBson(msg.time_stamp_is_present) // bool
    << "name"                  << toBson(msg.name)                  // etsi_its_spatem_ts_msgs/DescriptiveName
    << "name_is_present"       << toBson(msg.name_is_present)       // bool
    << "intersections"         << toBson(msg.intersections)         // etsi_its_spatem_ts_msgs/IntersectionStateList
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::SPATEM& msg) {
  return document{}
    << "header" << toBson(msg.header) // etsi_its_spatem_ts_msgs/ItsPduHeader
    << "spat"   << toBson(msg.spat)   // etsi_its_spatem_ts_msgs/SPAT
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::SignalGroupID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::SpeedAdvice& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::SpeedConfidenceDSRC& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::StationID& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::TimeChangeDetails& msg) {
  return document{}
    << "start_time"              << toBson(msg.start_time)              // etsi_its_spatem_ts_msgs/TimeMark
    << "start_time_is_present"   << toBson(msg.start_time_is_present)   // bool
    << "min_end_time"            << toBson(msg.min_end_time)            // etsi_its_spatem_ts_msgs/TimeMark
    << "max_end_time"            << toBson(msg.max_end_time)            // etsi_its_spatem_ts_msgs/TimeMark
    << "max_end_time_is_present" << toBson(msg.max_end_time_is_present) // bool
    << "likely_time"             << toBson(msg.likely_time)             // etsi_its_spatem_ts_msgs/TimeMark
    << "likely_time_is_present"  << toBson(msg.likely_time_is_present)  // bool
    << "confidence"              << toBson(msg.confidence)              // etsi_its_spatem_ts_msgs/TimeIntervalConfidence
    << "confidence_is_present"   << toBson(msg.confidence_is_present)   // bool
    << "next_time"               << toBson(msg.next_time)               // etsi_its_spatem_ts_msgs/TimeMark
    << "next_time_is_present"    << toBson(msg.next_time_is_present)    // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::TimeIntervalConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::TimeMark& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::WaitOnStopline& msg) {
  return document{}
    << "value" << toBson(msg.value) // bool
  << finalize;
}

value toBson(const etsi_its_spatem_ts_msgs::msg::ZoneLength& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::AccelerationChange& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::AccelerationChangeIndication& msg) {
  return document{}
    << "accel_or_decel"    << toBson(msg.accel_or_decel)    // etsi_its_vam_ts_msgs/AccelerationChange
    << "action_delta_time" << toBson(msg.action_delta_time) // etsi_its_vam_ts_msgs/DeltaTimeTenthOfSecond
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::AccelerationConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Altitude& msg) {
  return document{}
    << "altitude_value"      << toBson(msg.altitude_value)      // etsi_its_vam_ts_msgs/AltitudeValue
    << "altitude_confidence" << toBson(msg.altitude_confidence) // etsi_its_vam_ts_msgs/AltitudeConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::AltitudeConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::AltitudeValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::AngleConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::BasicContainer& msg) {
  return document{}
    << "station_type"       << toBson(msg.station_type)       // etsi_its_vam_ts_msgs/TrafficParticipantType
    << "reference_position" << toBson(msg.reference_position) // etsi_its_vam_ts_msgs/ReferencePositionWithConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CardinalNumber1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CartesianAngle& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "confidence" << toBson(msg.confidence) // etsi_its_vam_ts_msgs/AngleConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CartesianAngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CartesianCoordinate& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CartesianCoordinateSmall& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CartesianPosition3d& msg) {
  return document{}
    << "x_coordinate"            << toBson(msg.x_coordinate)            // etsi_its_vam_ts_msgs/CartesianCoordinate
    << "y_coordinate"            << toBson(msg.y_coordinate)            // etsi_its_vam_ts_msgs/CartesianCoordinate
    << "z_coordinate"            << toBson(msg.z_coordinate)            // etsi_its_vam_ts_msgs/CartesianCoordinate
    << "z_coordinate_is_present" << toBson(msg.z_coordinate_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CircularShape& msg) {
  return document{}
    << "shape_reference_point"            << toBson(msg.shape_reference_point)            // etsi_its_vam_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present" << toBson(msg.shape_reference_point_is_present) // bool
    << "radius"                           << toBson(msg.radius)                           // etsi_its_vam_ts_msgs/StandardLength12b
    << "height"                           << toBson(msg.height)                           // etsi_its_vam_ts_msgs/StandardLength12b
    << "height_is_present"                << toBson(msg.height_is_present)                // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ClusterBreakupInfo& msg) {
  return document{}
    << "cluster_breakup_reason" << toBson(msg.cluster_breakup_reason) // etsi_its_vam_ts_msgs/ClusterBreakupReason
    << "breakup_time"           << toBson(msg.breakup_time)           // etsi_its_vam_ts_msgs/DeltaTimeQuarterSecond
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ClusterBreakupReason& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ClusterJoinInfo& msg) {
  return document{}
    << "cluster_id" << toBson(msg.cluster_id) // etsi_its_vam_ts_msgs/Identifier1B
    << "join_time"  << toBson(msg.join_time)  // etsi_its_vam_ts_msgs/DeltaTimeQuarterSecond
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ClusterLeaveInfo& msg) {
  return document{}
    << "cluster_id"           << toBson(msg.cluster_id)           // etsi_its_vam_ts_msgs/Identifier1B
    << "cluster_leave_reason" << toBson(msg.cluster_leave_reason) // etsi_its_vam_ts_msgs/ClusterLeaveReason
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ClusterLeaveReason& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Curvature& msg) {
  return document{}
    << "curvature_value"      << toBson(msg.curvature_value)      // etsi_its_vam_ts_msgs/CurvatureValue
    << "curvature_confidence" << toBson(msg.curvature_confidence) // etsi_its_vam_ts_msgs/CurvatureConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CurvatureCalculationMode& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CurvatureConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::CurvatureValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::DeltaAltitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::DeltaLatitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::DeltaLongitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::DeltaReferencePosition& msg) {
  return document{}
    << "delta_latitude"  << toBson(msg.delta_latitude)  // etsi_its_vam_ts_msgs/DeltaLatitude
    << "delta_longitude" << toBson(msg.delta_longitude) // etsi_its_vam_ts_msgs/DeltaLongitude
    << "delta_altitude"  << toBson(msg.delta_altitude)  // etsi_its_vam_ts_msgs/DeltaAltitude
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::DeltaTimeQuarterSecond& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::DeltaTimeTenthOfSecond& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::EllipticalShape& msg) {
  return document{}
    << "shape_reference_point"            << toBson(msg.shape_reference_point)            // etsi_its_vam_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present" << toBson(msg.shape_reference_point_is_present) // bool
    << "semi_major_axis_length"           << toBson(msg.semi_major_axis_length)           // etsi_its_vam_ts_msgs/StandardLength12b
    << "semi_minor_axis_length"           << toBson(msg.semi_minor_axis_length)           // etsi_its_vam_ts_msgs/StandardLength12b
    << "orientation"                      << toBson(msg.orientation)                      // etsi_its_vam_ts_msgs/Wgs84AngleValue
    << "orientation_is_present"           << toBson(msg.orientation_is_present)           // bool
    << "height"                           << toBson(msg.height)                           // etsi_its_vam_ts_msgs/StandardLength12b
    << "height_is_present"                << toBson(msg.height_is_present)                // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ExteriorLights& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::GeneralizedLanePosition& msg) {
  return document{}
    << "choice"                    << toBson(msg.choice)                    // uint8
    << "traffic_lane_position"     << toBson(msg.traffic_lane_position)     // etsi_its_vam_ts_msgs/LanePosition
    << "non_traffic_lane_position" << toBson(msg.non_traffic_lane_position) // etsi_its_vam_ts_msgs/LanePositionAndType
    << "traffic_island_position"   << toBson(msg.traffic_island_position)   // etsi_its_vam_ts_msgs/TrafficIslandPosition
    << "map_position"              << toBson(msg.map_position)              // etsi_its_vam_ts_msgs/MapPosition
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::GenerationDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::HeadingChangeIndication& msg) {
  return document{}
    << "direction"         << toBson(msg.direction)         // etsi_its_vam_ts_msgs/TurningDirection
    << "action_delta_time" << toBson(msg.action_delta_time) // etsi_its_vam_ts_msgs/DeltaTimeTenthOfSecond
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::HeadingValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Identifier1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Identifier2B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::IntersectionReferenceId& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_vam_ts_msgs/Identifier2B
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_vam_ts_msgs/Identifier2B
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ItsPduHeader& msg) {
  return document{}
    << "protocol_version" << toBson(msg.protocol_version) // etsi_its_vam_ts_msgs/OrdinalNumber1B
    << "message_id"       << toBson(msg.message_id)       // etsi_its_vam_ts_msgs/MessageId
    << "station_id"       << toBson(msg.station_id)       // etsi_its_vam_ts_msgs/StationId
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ItsPduHeaderVam& msg) {
  return document{}
    << "value" << toBson(msg.value) // etsi_its_vam_ts_msgs/ItsPduHeader
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LanePosition& msg) {
  return document{}
    << "value" << toBson(msg.value) // int8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LanePositionAndType& msg) {
  return document{}
    << "transversal_position" << toBson(msg.transversal_position) // etsi_its_vam_ts_msgs/LanePosition
    << "lane_type"            << toBson(msg.lane_type)            // etsi_its_vam_ts_msgs/LaneType
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LaneType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LateralAcceleration& msg) {
  return document{}
    << "lateral_acceleration_value"      << toBson(msg.lateral_acceleration_value)      // etsi_its_vam_ts_msgs/LateralAccelerationValue
    << "lateral_acceleration_confidence" << toBson(msg.lateral_acceleration_confidence) // etsi_its_vam_ts_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LateralAccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Latitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Longitude& msg) {
  return document{}
    << "value" << toBson(msg.value) // int32
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalAcceleration& msg) {
  return document{}
    << "longitudinal_acceleration_value"      << toBson(msg.longitudinal_acceleration_value)      // etsi_its_vam_ts_msgs/LongitudinalAccelerationValue
    << "longitudinal_acceleration_confidence" << toBson(msg.longitudinal_acceleration_confidence) // etsi_its_vam_ts_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalAccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalLanePosition& msg) {
  return document{}
    << "longitudinal_lane_position_value"      << toBson(msg.longitudinal_lane_position_value)      // etsi_its_vam_ts_msgs/LongitudinalLanePositionValue
    << "longitudinal_lane_position_confidence" << toBson(msg.longitudinal_lane_position_confidence) // etsi_its_vam_ts_msgs/LongitudinalLanePositionConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalLanePositionConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::LongitudinalLanePositionValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::MapPosition& msg) {
  return document{}
    << "map_reference"                         << toBson(msg.map_reference)                         // etsi_its_vam_ts_msgs/MapReference
    << "map_reference_is_present"              << toBson(msg.map_reference_is_present)              // bool
    << "lane_id"                               << toBson(msg.lane_id)                               // etsi_its_vam_ts_msgs/Identifier1B
    << "lane_id_is_present"                    << toBson(msg.lane_id_is_present)                    // bool
    << "connection_id"                         << toBson(msg.connection_id)                         // etsi_its_vam_ts_msgs/Identifier1B
    << "connection_id_is_present"              << toBson(msg.connection_id_is_present)              // bool
    << "longitudinal_lane_position"            << toBson(msg.longitudinal_lane_position)            // etsi_its_vam_ts_msgs/LongitudinalLanePosition
    << "longitudinal_lane_position_is_present" << toBson(msg.longitudinal_lane_position_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::MapReference& msg) {
  return document{}
    << "choice"       << toBson(msg.choice)       // uint8
    << "roadsegment"  << toBson(msg.roadsegment)  // etsi_its_vam_ts_msgs/RoadSegmentReferenceId
    << "intersection" << toBson(msg.intersection) // etsi_its_vam_ts_msgs/IntersectionReferenceId
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::MessageId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::OrdinalNumber1B& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PathDeltaTime& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PathHistory& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_vam_ts_msgs/PathPoint[]
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PathPoint& msg) {
  return document{}
    << "path_position"              << toBson(msg.path_position)              // etsi_its_vam_ts_msgs/DeltaReferencePosition
    << "path_delta_time"            << toBson(msg.path_delta_time)            // etsi_its_vam_ts_msgs/PathDeltaTime
    << "path_delta_time_is_present" << toBson(msg.path_delta_time_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PathPointPredicted& msg) {
  return document{}
    << "delta_latitude"                            << toBson(msg.delta_latitude)                            // etsi_its_vam_ts_msgs/DeltaLatitude
    << "delta_longitude"                           << toBson(msg.delta_longitude)                           // etsi_its_vam_ts_msgs/DeltaLongitude
    << "horizontal_position_confidence"            << toBson(msg.horizontal_position_confidence)            // etsi_its_vam_ts_msgs/PosConfidenceEllipse
    << "horizontal_position_confidence_is_present" << toBson(msg.horizontal_position_confidence_is_present) // bool
    << "delta_altitude"                            << toBson(msg.delta_altitude)                            // etsi_its_vam_ts_msgs/DeltaAltitude
    << "altitude_confidence"                       << toBson(msg.altitude_confidence)                       // etsi_its_vam_ts_msgs/AltitudeConfidence
    << "path_delta_time"                           << toBson(msg.path_delta_time)                           // etsi_its_vam_ts_msgs/DeltaTimeTenthOfSecond
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PathPredicted& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_vam_ts_msgs/PathPointPredicted[]
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PolygonalShape& msg) {
  return document{}
    << "shape_reference_point"            << toBson(msg.shape_reference_point)            // etsi_its_vam_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present" << toBson(msg.shape_reference_point_is_present) // bool
    << "polygon"                          << toBson(msg.polygon)                          // etsi_its_vam_ts_msgs/SequenceOfCartesianPosition3d
    << "height"                           << toBson(msg.height)                           // etsi_its_vam_ts_msgs/StandardLength12b
    << "height_is_present"                << toBson(msg.height_is_present)                // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PosConfidenceEllipse& msg) {
  return document{}
    << "semi_major_confidence"  << toBson(msg.semi_major_confidence)  // etsi_its_vam_ts_msgs/SemiAxisLength
    << "semi_minor_confidence"  << toBson(msg.semi_minor_confidence)  // etsi_its_vam_ts_msgs/SemiAxisLength
    << "semi_major_orientation" << toBson(msg.semi_major_orientation) // etsi_its_vam_ts_msgs/HeadingValue
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::PositionConfidenceEllipse& msg) {
  return document{}
    << "semi_major_axis_length"      << toBson(msg.semi_major_axis_length)      // etsi_its_vam_ts_msgs/SemiAxisLength
    << "semi_minor_axis_length"      << toBson(msg.semi_minor_axis_length)      // etsi_its_vam_ts_msgs/SemiAxisLength
    << "semi_major_axis_orientation" << toBson(msg.semi_major_axis_orientation) // etsi_its_vam_ts_msgs/Wgs84AngleValue
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::RadialShape& msg) {
  return document{}
    << "shape_reference_point"                     << toBson(msg.shape_reference_point)                     // etsi_its_vam_ts_msgs/CartesianPosition3d
    << "shape_reference_point_is_present"          << toBson(msg.shape_reference_point_is_present)          // bool
    << "range"                                     << toBson(msg.range)                                     // etsi_its_vam_ts_msgs/StandardLength12b
    << "stationary_horizontal_opening_angle_start" << toBson(msg.stationary_horizontal_opening_angle_start) // etsi_its_vam_ts_msgs/Wgs84AngleValue
    << "stationary_horizontal_opening_angle_end"   << toBson(msg.stationary_horizontal_opening_angle_end)   // etsi_its_vam_ts_msgs/Wgs84AngleValue
    << "vertical_opening_angle_start"              << toBson(msg.vertical_opening_angle_start)              // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_start_is_present"   << toBson(msg.vertical_opening_angle_start_is_present)   // bool
    << "vertical_opening_angle_end"                << toBson(msg.vertical_opening_angle_end)                // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_end_is_present"     << toBson(msg.vertical_opening_angle_end_is_present)     // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::RadialShapeDetails& msg) {
  return document{}
    << "range"                                   << toBson(msg.range)                                   // etsi_its_vam_ts_msgs/StandardLength12b
    << "horizontal_opening_angle_start"          << toBson(msg.horizontal_opening_angle_start)          // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "horizontal_opening_angle_end"            << toBson(msg.horizontal_opening_angle_end)            // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_start"            << toBson(msg.vertical_opening_angle_start)            // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_start_is_present" << toBson(msg.vertical_opening_angle_start_is_present) // bool
    << "vertical_opening_angle_end"              << toBson(msg.vertical_opening_angle_end)              // etsi_its_vam_ts_msgs/CartesianAngleValue
    << "vertical_opening_angle_end_is_present"   << toBson(msg.vertical_opening_angle_end_is_present)   // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::RadialShapes& msg) {
  return document{}
    << "ref_point_id"            << toBson(msg.ref_point_id)            // etsi_its_vam_ts_msgs/Identifier1B
    << "x_coordinate"            << toBson(msg.x_coordinate)            // etsi_its_vam_ts_msgs/CartesianCoordinateSmall
    << "y_coordinate"            << toBson(msg.y_coordinate)            // etsi_its_vam_ts_msgs/CartesianCoordinateSmall
    << "z_coordinate"            << toBson(msg.z_coordinate)            // etsi_its_vam_ts_msgs/CartesianCoordinateSmall
    << "z_coordinate_is_present" << toBson(msg.z_coordinate_is_present) // bool
    << "radial_shapes_list"      << toBson(msg.radial_shapes_list)      // etsi_its_vam_ts_msgs/RadialShapesList
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::RadialShapesList& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_vam_ts_msgs/RadialShapeDetails[]
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::RectangularShape& msg) {
  return document{}
    << "center_point"            << toBson(msg.center_point)            // etsi_its_vam_ts_msgs/CartesianPosition3d
    << "center_point_is_present" << toBson(msg.center_point_is_present) // bool
    << "semi_length"             << toBson(msg.semi_length)             // etsi_its_vam_ts_msgs/StandardLength12b
    << "semi_breadth"            << toBson(msg.semi_breadth)            // etsi_its_vam_ts_msgs/StandardLength12b
    << "orientation"             << toBson(msg.orientation)             // etsi_its_vam_ts_msgs/Wgs84AngleValue
    << "orientation_is_present"  << toBson(msg.orientation_is_present)  // bool
    << "height"                  << toBson(msg.height)                  // etsi_its_vam_ts_msgs/StandardLength12b
    << "height_is_present"       << toBson(msg.height_is_present)       // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::ReferencePositionWithConfidence& msg) {
  return document{}
    << "latitude"                    << toBson(msg.latitude)                    // etsi_its_vam_ts_msgs/Latitude
    << "longitude"                   << toBson(msg.longitude)                   // etsi_its_vam_ts_msgs/Longitude
    << "position_confidence_ellipse" << toBson(msg.position_confidence_ellipse) // etsi_its_vam_ts_msgs/PositionConfidenceEllipse
    << "altitude"                    << toBson(msg.altitude)                    // etsi_its_vam_ts_msgs/Altitude
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::RoadSegmentReferenceId& msg) {
  return document{}
    << "region"            << toBson(msg.region)            // etsi_its_vam_ts_msgs/Identifier2B
    << "region_is_present" << toBson(msg.region_is_present) // bool
    << "id"                << toBson(msg.id)                // etsi_its_vam_ts_msgs/Identifier2B
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SafeDistanceIndication& msg) {
  return document{}
    << "subject_station"              << toBson(msg.subject_station)              // etsi_its_vam_ts_msgs/StationId
    << "subject_station_is_present"   << toBson(msg.subject_station_is_present)   // bool
    << "safe_distance_indicator"      << toBson(msg.safe_distance_indicator)      // etsi_its_vam_ts_msgs/SafeDistanceIndicator
    << "time_to_collision"            << toBson(msg.time_to_collision)            // etsi_its_vam_ts_msgs/DeltaTimeTenthOfSecond
    << "time_to_collision_is_present" << toBson(msg.time_to_collision_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SafeDistanceIndicator& msg) {
  return document{}
    << "value" << toBson(msg.value) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SemiAxisLength& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SequenceOfCartesianPosition3d& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_vam_ts_msgs/CartesianPosition3d[]
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SequenceOfSafeDistanceIndication& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_vam_ts_msgs/SafeDistanceIndication[]
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SequenceOfTrajectoryInterceptionIndication& msg) {
  return document{}
    << "array" << toBson(msg.array) // etsi_its_vam_ts_msgs/TrajectoryInterceptionIndication[]
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Shape& msg) {
  return document{}
    << "choice"        << toBson(msg.choice)        // uint8
    << "rectangular"   << toBson(msg.rectangular)   // etsi_its_vam_ts_msgs/RectangularShape
    << "circular"      << toBson(msg.circular)      // etsi_its_vam_ts_msgs/CircularShape
    << "polygonal"     << toBson(msg.polygonal)     // etsi_its_vam_ts_msgs/PolygonalShape
    << "elliptical"    << toBson(msg.elliptical)    // etsi_its_vam_ts_msgs/EllipticalShape
    << "radial"        << toBson(msg.radial)        // etsi_its_vam_ts_msgs/RadialShape
    << "radial_shapes" << toBson(msg.radial_shapes) // etsi_its_vam_ts_msgs/RadialShapes
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Speed& msg) {
  return document{}
    << "speed_value"      << toBson(msg.speed_value)      // etsi_its_vam_ts_msgs/SpeedValue
    << "speed_confidence" << toBson(msg.speed_confidence) // etsi_its_vam_ts_msgs/SpeedConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SpeedConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::SpeedValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::StabilityChangeIndication& msg) {
  return document{}
    << "loss_probability"  << toBson(msg.loss_probability)  // etsi_its_vam_ts_msgs/StabilityLossProbability
    << "action_delta_time" << toBson(msg.action_delta_time) // etsi_its_vam_ts_msgs/DeltaTimeTenthOfSecond
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::StabilityLossProbability& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::StandardLength12b& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::StationId& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint32
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::TrafficIslandPosition& msg) {
  return document{}
    << "one_side"   << toBson(msg.one_side)   // etsi_its_vam_ts_msgs/LanePositionAndType
    << "other_side" << toBson(msg.other_side) // etsi_its_vam_ts_msgs/LanePositionAndType
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::TrafficParticipantType& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::TrajectoryInterceptionConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::TrajectoryInterceptionIndication& msg) {
  return document{}
    << "subject_station"                               << toBson(msg.subject_station)                               // etsi_its_vam_ts_msgs/StationId
    << "subject_station_is_present"                    << toBson(msg.subject_station_is_present)                    // bool
    << "trajectory_interception_probability"           << toBson(msg.trajectory_interception_probability)           // etsi_its_vam_ts_msgs/TrajectoryInterceptionProbability
    << "trajectory_interception_confidence"            << toBson(msg.trajectory_interception_confidence)            // etsi_its_vam_ts_msgs/TrajectoryInterceptionConfidence
    << "trajectory_interception_confidence_is_present" << toBson(msg.trajectory_interception_confidence_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::TrajectoryInterceptionProbability& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::TurningDirection& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VAM& msg) {
  return document{}
    << "header" << toBson(msg.header) // etsi_its_vam_ts_msgs/ItsPduHeaderVam
    << "vam"    << toBson(msg.vam)    // etsi_its_vam_ts_msgs/VruAwareness
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VamParameters& msg) {
  return document{}
    << "basic_container"                              << toBson(msg.basic_container)                              // etsi_its_vam_ts_msgs/BasicContainer
    << "vru_high_frequency_container"                 << toBson(msg.vru_high_frequency_container)                 // etsi_its_vam_ts_msgs/VruHighFrequencyContainer
    << "vru_low_frequency_container"                  << toBson(msg.vru_low_frequency_container)                  // etsi_its_vam_ts_msgs/VruLowFrequencyContainer
    << "vru_low_frequency_container_is_present"       << toBson(msg.vru_low_frequency_container_is_present)       // bool
    << "vru_cluster_information_container"            << toBson(msg.vru_cluster_information_container)            // etsi_its_vam_ts_msgs/VruClusterInformationContainer
    << "vru_cluster_information_container_is_present" << toBson(msg.vru_cluster_information_container_is_present) // bool
    << "vru_cluster_operation_container"              << toBson(msg.vru_cluster_operation_container)              // etsi_its_vam_ts_msgs/VruClusterOperationContainer
    << "vru_cluster_operation_container_is_present"   << toBson(msg.vru_cluster_operation_container_is_present)   // bool
    << "vru_motion_prediction_container"              << toBson(msg.vru_motion_prediction_container)              // etsi_its_vam_ts_msgs/VruMotionPredictionContainer
    << "vru_motion_prediction_container_is_present"   << toBson(msg.vru_motion_prediction_container_is_present)   // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VerticalAcceleration& msg) {
  return document{}
    << "vertical_acceleration_value"      << toBson(msg.vertical_acceleration_value)      // etsi_its_vam_ts_msgs/VerticalAccelerationValue
    << "vertical_acceleration_confidence" << toBson(msg.vertical_acceleration_confidence) // etsi_its_vam_ts_msgs/AccelerationConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VerticalAccelerationValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruAwareness& msg) {
  return document{}
    << "generation_delta_time" << toBson(msg.generation_delta_time) // etsi_its_vam_ts_msgs/GenerationDeltaTime
    << "vam_parameters"        << toBson(msg.vam_parameters)        // etsi_its_vam_ts_msgs/VamParameters
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterInformation& msg) {
  return document{}
    << "cluster_id"                            << toBson(msg.cluster_id)                            // etsi_its_vam_ts_msgs/Identifier1B
    << "cluster_id_is_present"                 << toBson(msg.cluster_id_is_present)                 // bool
    << "cluster_bounding_box_shape"            << toBson(msg.cluster_bounding_box_shape)            // etsi_its_vam_ts_msgs/Shape
    << "cluster_bounding_box_shape_is_present" << toBson(msg.cluster_bounding_box_shape_is_present) // bool
    << "cluster_cardinality_size"              << toBson(msg.cluster_cardinality_size)              // etsi_its_vam_ts_msgs/CardinalNumber1B
    << "cluster_profiles"                      << toBson(msg.cluster_profiles)                      // etsi_its_vam_ts_msgs/VruClusterProfiles
    << "cluster_profiles_is_present"           << toBson(msg.cluster_profiles_is_present)           // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterInformationContainer& msg) {
  return document{}
    << "vru_cluster_information" << toBson(msg.vru_cluster_information) // etsi_its_vam_ts_msgs/VruClusterInformation
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterOperationContainer& msg) {
  return document{}
    << "cluster_join_info"                      << toBson(msg.cluster_join_info)                      // etsi_its_vam_ts_msgs/ClusterJoinInfo
    << "cluster_join_info_is_present"           << toBson(msg.cluster_join_info_is_present)           // bool
    << "cluster_leave_info"                     << toBson(msg.cluster_leave_info)                     // etsi_its_vam_ts_msgs/ClusterLeaveInfo
    << "cluster_leave_info_is_present"          << toBson(msg.cluster_leave_info_is_present)          // bool
    << "cluster_breakup_info"                   << toBson(msg.cluster_breakup_info)                   // etsi_its_vam_ts_msgs/ClusterBreakupInfo
    << "cluster_breakup_info_is_present"        << toBson(msg.cluster_breakup_info_is_present)        // bool
    << "cluster_id_change_time_info"            << toBson(msg.cluster_id_change_time_info)            // etsi_its_vam_ts_msgs/DeltaTimeQuarterSecond
    << "cluster_id_change_time_info_is_present" << toBson(msg.cluster_id_change_time_info_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruClusterProfiles& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruDeviceUsage& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruEnvironment& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruExteriorLights& msg) {
  return document{}
    << "vehicular"    << toBson(msg.vehicular)    // etsi_its_vam_ts_msgs/ExteriorLights
    << "vru_specific" << toBson(msg.vru_specific) // etsi_its_vam_ts_msgs/VruSpecificExteriorLights
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruHighFrequencyContainer& msg) {
  return document{}
    << "heading"                               << toBson(msg.heading)                               // etsi_its_vam_ts_msgs/Wgs84Angle
    << "speed"                                 << toBson(msg.speed)                                 // etsi_its_vam_ts_msgs/Speed
    << "longitudinal_acceleration"             << toBson(msg.longitudinal_acceleration)             // etsi_its_vam_ts_msgs/LongitudinalAcceleration
    << "curvature"                             << toBson(msg.curvature)                             // etsi_its_vam_ts_msgs/Curvature
    << "curvature_is_present"                  << toBson(msg.curvature_is_present)                  // bool
    << "curvature_calculation_mode"            << toBson(msg.curvature_calculation_mode)            // etsi_its_vam_ts_msgs/CurvatureCalculationMode
    << "curvature_calculation_mode_is_present" << toBson(msg.curvature_calculation_mode_is_present) // bool
    << "yaw_rate"                              << toBson(msg.yaw_rate)                              // etsi_its_vam_ts_msgs/YawRate
    << "yaw_rate_is_present"                   << toBson(msg.yaw_rate_is_present)                   // bool
    << "lateral_acceleration"                  << toBson(msg.lateral_acceleration)                  // etsi_its_vam_ts_msgs/LateralAcceleration
    << "lateral_acceleration_is_present"       << toBson(msg.lateral_acceleration_is_present)       // bool
    << "vertical_acceleration"                 << toBson(msg.vertical_acceleration)                 // etsi_its_vam_ts_msgs/VerticalAcceleration
    << "vertical_acceleration_is_present"      << toBson(msg.vertical_acceleration_is_present)      // bool
    << "vru_lane_position"                     << toBson(msg.vru_lane_position)                     // etsi_its_vam_ts_msgs/GeneralizedLanePosition
    << "vru_lane_position_is_present"          << toBson(msg.vru_lane_position_is_present)          // bool
    << "environment"                           << toBson(msg.environment)                           // etsi_its_vam_ts_msgs/VruEnvironment
    << "environment_is_present"                << toBson(msg.environment_is_present)                // bool
    << "movement_control"                      << toBson(msg.movement_control)                      // etsi_its_vam_ts_msgs/VruMovementControl
    << "movement_control_is_present"           << toBson(msg.movement_control_is_present)           // bool
    << "orientation"                           << toBson(msg.orientation)                           // etsi_its_vam_ts_msgs/Wgs84Angle
    << "orientation_is_present"                << toBson(msg.orientation_is_present)                // bool
    << "roll_angle"                            << toBson(msg.roll_angle)                            // etsi_its_vam_ts_msgs/CartesianAngle
    << "roll_angle_is_present"                 << toBson(msg.roll_angle_is_present)                 // bool
    << "device_usage"                          << toBson(msg.device_usage)                          // etsi_its_vam_ts_msgs/VruDeviceUsage
    << "device_usage_is_present"               << toBson(msg.device_usage_is_present)               // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruLowFrequencyContainer& msg) {
  return document{}
    << "profile_and_subprofile"     << toBson(msg.profile_and_subprofile)     // etsi_its_vam_ts_msgs/VruProfileAndSubprofile
    << "size_class"                 << toBson(msg.size_class)                 // etsi_its_vam_ts_msgs/VruSizeClass
    << "size_class_is_present"      << toBson(msg.size_class_is_present)      // bool
    << "exterior_lights"            << toBson(msg.exterior_lights)            // etsi_its_vam_ts_msgs/VruExteriorLights
    << "exterior_lights_is_present" << toBson(msg.exterior_lights_is_present) // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruMotionPredictionContainer& msg) {
  return document{}
    << "path_history"                                  << toBson(msg.path_history)                                  // etsi_its_vam_ts_msgs/PathHistory
    << "path_history_is_present"                       << toBson(msg.path_history_is_present)                       // bool
    << "path_prediction"                               << toBson(msg.path_prediction)                               // etsi_its_vam_ts_msgs/PathPredicted
    << "path_prediction_is_present"                    << toBson(msg.path_prediction_is_present)                    // bool
    << "safe_distance"                                 << toBson(msg.safe_distance)                                 // etsi_its_vam_ts_msgs/SequenceOfSafeDistanceIndication
    << "safe_distance_is_present"                      << toBson(msg.safe_distance_is_present)                      // bool
    << "trajectory_interception_indication"            << toBson(msg.trajectory_interception_indication)            // etsi_its_vam_ts_msgs/SequenceOfTrajectoryInterceptionIndication
    << "trajectory_interception_indication_is_present" << toBson(msg.trajectory_interception_indication_is_present) // bool
    << "acceleration_change_indication"                << toBson(msg.acceleration_change_indication)                // etsi_its_vam_ts_msgs/AccelerationChangeIndication
    << "acceleration_change_indication_is_present"     << toBson(msg.acceleration_change_indication_is_present)     // bool
    << "heading_change_indication"                     << toBson(msg.heading_change_indication)                     // etsi_its_vam_ts_msgs/HeadingChangeIndication
    << "heading_change_indication_is_present"          << toBson(msg.heading_change_indication_is_present)          // bool
    << "stability_change_indication"                   << toBson(msg.stability_change_indication)                   // etsi_its_vam_ts_msgs/StabilityChangeIndication
    << "stability_change_indication_is_present"        << toBson(msg.stability_change_indication_is_present)        // bool
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruMovementControl& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruProfileAndSubprofile& msg) {
  return document{}
    << "choice"                          << toBson(msg.choice)                          // uint8
    << "pedestrian"                      << toBson(msg.pedestrian)                      // etsi_its_vam_ts_msgs/VruSubProfilePedestrian
    << "bicyclist_and_light_vru_vehicle" << toBson(msg.bicyclist_and_light_vru_vehicle) // etsi_its_vam_ts_msgs/VruSubProfileBicyclist
    << "motorcyclist"                    << toBson(msg.motorcyclist)                    // etsi_its_vam_ts_msgs/VruSubProfileMotorcyclist
    << "animal"                          << toBson(msg.animal)                          // etsi_its_vam_ts_msgs/VruSubProfileAnimal
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruSizeClass& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruSpecificExteriorLights& msg) {
  return document{}
    << "value"       << toBson(msg.value)       // uint8[]
    << "bits_unused" << toBson(msg.bits_unused) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfileAnimal& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfileBicyclist& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfileMotorcyclist& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::VruSubProfilePedestrian& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Wgs84Angle& msg) {
  return document{}
    << "value"      << toBson(msg.value)      // etsi_its_vam_ts_msgs/Wgs84AngleValue
    << "confidence" << toBson(msg.confidence) // etsi_its_vam_ts_msgs/Wgs84AngleConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Wgs84AngleConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::Wgs84AngleValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint16
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::YawRate& msg) {
  return document{}
    << "yaw_rate_value"      << toBson(msg.yaw_rate_value)      // etsi_its_vam_ts_msgs/YawRateValue
    << "yaw_rate_confidence" << toBson(msg.yaw_rate_confidence) // etsi_its_vam_ts_msgs/YawRateConfidence
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::YawRateConfidence& msg) {
  return document{}
    << "value" << toBson(msg.value) // uint8
  << finalize;
}

value toBson(const etsi_its_vam_ts_msgs::msg::YawRateValue& msg) {
  return document{}
    << "value" << toBson(msg.value) // int16
  << finalize;
}

value toBson(const geometry_msgs::msg::Accel& msg) {
  return document{}
    << "linear"  << toBson(msg.linear)  // geometry_msgs/Vector3
    << "angular" << toBson(msg.angular) // geometry_msgs/Vector3
  << finalize;
}

value toBson(const geometry_msgs::msg::AccelStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "accel"  << toBson(msg.accel)  // geometry_msgs/Accel
  << finalize;
}

value toBson(const geometry_msgs::msg::AccelWithCovariance& msg) {
  return document{}
    << "accel"      << toBson(msg.accel)      // geometry_msgs/Accel
    << "covariance" << toBson(msg.covariance) // float64[36]
  << finalize;
}

value toBson(const geometry_msgs::msg::AccelWithCovarianceStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "accel"  << toBson(msg.accel)  // geometry_msgs/AccelWithCovariance
  << finalize;
}

value toBson(const geometry_msgs::msg::Inertia& msg) {
  return document{}
    << "m"   << toBson(msg.m)   // float64
    << "com" << toBson(msg.com) // geometry_msgs/Vector3
    << "ixx" << toBson(msg.ixx) // float64
    << "ixy" << toBson(msg.ixy) // float64
    << "ixz" << toBson(msg.ixz) // float64
    << "iyy" << toBson(msg.iyy) // float64
    << "iyz" << toBson(msg.iyz) // float64
    << "izz" << toBson(msg.izz) // float64
  << finalize;
}

value toBson(const geometry_msgs::msg::InertiaStamped& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "inertia" << toBson(msg.inertia) // geometry_msgs/Inertia
  << finalize;
}

value toBson(const geometry_msgs::msg::Point& msg) {
  return document{}
    << "x" << toBson(msg.x) // float64
    << "y" << toBson(msg.y) // float64
    << "z" << toBson(msg.z) // float64
  << finalize;
}

value toBson(const geometry_msgs::msg::Point32& msg) {
  return document{}
    << "x" << toBson(msg.x) // float32
    << "y" << toBson(msg.y) // float32
    << "z" << toBson(msg.z) // float32
  << finalize;
}

value toBson(const geometry_msgs::msg::PointStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "point"  << toBson(msg.point)  // geometry_msgs/Point
  << finalize;
}

value toBson(const geometry_msgs::msg::Polygon& msg) {
  return document{}
    << "points" << toBson(msg.points) // geometry_msgs/Point32[]
  << finalize;
}

value toBson(const geometry_msgs::msg::PolygonStamped& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "polygon" << toBson(msg.polygon) // geometry_msgs/Polygon
  << finalize;
}

value toBson(const geometry_msgs::msg::Pose& msg) {
  return document{}
    << "position"    << toBson(msg.position)    // geometry_msgs/Point
    << "orientation" << toBson(msg.orientation) // geometry_msgs/Quaternion
  << finalize;
}

value toBson(const geometry_msgs::msg::Pose2D& msg) {
  return document{}
    << "x"     << toBson(msg.x)     // float64
    << "y"     << toBson(msg.y)     // float64
    << "theta" << toBson(msg.theta) // float64
  << finalize;
}

value toBson(const geometry_msgs::msg::PoseArray& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "poses"  << toBson(msg.poses)  // geometry_msgs/Pose[]
  << finalize;
}

value toBson(const geometry_msgs::msg::PoseStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "pose"   << toBson(msg.pose)   // geometry_msgs/Pose
  << finalize;
}

value toBson(const geometry_msgs::msg::PoseWithCovariance& msg) {
  return document{}
    << "pose"       << toBson(msg.pose)       // geometry_msgs/Pose
    << "covariance" << toBson(msg.covariance) // float64[36]
  << finalize;
}

value toBson(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "pose"   << toBson(msg.pose)   // geometry_msgs/PoseWithCovariance
  << finalize;
}

value toBson(const geometry_msgs::msg::Quaternion& msg) {
  return document{}
    << "x" << toBson(msg.x) // float64
    << "y" << toBson(msg.y) // float64
    << "z" << toBson(msg.z) // float64
    << "w" << toBson(msg.w) // float64
  << finalize;
}

value toBson(const geometry_msgs::msg::QuaternionStamped& msg) {
  return document{}
    << "header"     << toBson(msg.header)     // std_msgs/Header
    << "quaternion" << toBson(msg.quaternion) // geometry_msgs/Quaternion
  << finalize;
}

value toBson(const geometry_msgs::msg::Transform& msg) {
  return document{}
    << "translation" << toBson(msg.translation) // geometry_msgs/Vector3
    << "rotation"    << toBson(msg.rotation)    // geometry_msgs/Quaternion
  << finalize;
}

value toBson(const geometry_msgs::msg::TransformStamped& msg) {
  return document{}
    << "header"         << toBson(msg.header)         // std_msgs/Header
    << "child_frame_id" << toBson(msg.child_frame_id) // string
    << "transform"      << toBson(msg.transform)      // geometry_msgs/Transform
  << finalize;
}

value toBson(const geometry_msgs::msg::Twist& msg) {
  return document{}
    << "linear"  << toBson(msg.linear)  // geometry_msgs/Vector3
    << "angular" << toBson(msg.angular) // geometry_msgs/Vector3
  << finalize;
}

value toBson(const geometry_msgs::msg::TwistStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "twist"  << toBson(msg.twist)  // geometry_msgs/Twist
  << finalize;
}

value toBson(const geometry_msgs::msg::TwistWithCovariance& msg) {
  return document{}
    << "twist"      << toBson(msg.twist)      // geometry_msgs/Twist
    << "covariance" << toBson(msg.covariance) // float64[36]
  << finalize;
}

value toBson(const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "twist"  << toBson(msg.twist)  // geometry_msgs/TwistWithCovariance
  << finalize;
}

value toBson(const geometry_msgs::msg::Vector3& msg) {
  return document{}
    << "x" << toBson(msg.x) // float64
    << "y" << toBson(msg.y) // float64
    << "z" << toBson(msg.z) // float64
  << finalize;
}

value toBson(const geometry_msgs::msg::Vector3Stamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "vector" << toBson(msg.vector) // geometry_msgs/Vector3
  << finalize;
}

value toBson(const geometry_msgs::msg::VelocityStamped& msg) {
  return document{}
    << "header"             << toBson(msg.header)             // std_msgs/Header
    << "body_frame_id"      << toBson(msg.body_frame_id)      // string
    << "reference_frame_id" << toBson(msg.reference_frame_id) // string
    << "velocity"           << toBson(msg.velocity)           // geometry_msgs/Twist
  << finalize;
}

value toBson(const geometry_msgs::msg::Wrench& msg) {
  return document{}
    << "force"  << toBson(msg.force)  // geometry_msgs/Vector3
    << "torque" << toBson(msg.torque) // geometry_msgs/Vector3
  << finalize;
}

value toBson(const geometry_msgs::msg::WrenchStamped& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "wrench" << toBson(msg.wrench) // geometry_msgs/Wrench
  << finalize;
}

value toBson(const nav_msgs::msg::GridCells& msg) {
  return document{}
    << "header"      << toBson(msg.header)      // std_msgs/Header
    << "cell_width"  << toBson(msg.cell_width)  // float32
    << "cell_height" << toBson(msg.cell_height) // float32
    << "cells"       << toBson(msg.cells)       // geometry_msgs/Point[]
  << finalize;
}

value toBson(const nav_msgs::msg::MapMetaData& msg) {
  return document{}
    << "map_load_time" << toBson(msg.map_load_time) // builtin_interfaces/Time
    << "resolution"    << toBson(msg.resolution)    // float32
    << "width"         << toBson(msg.width)         // uint32
    << "height"        << toBson(msg.height)        // uint32
    << "origin"        << toBson(msg.origin)        // geometry_msgs/Pose
  << finalize;
}

value toBson(const nav_msgs::msg::OccupancyGrid& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "info"   << toBson(msg.info)   // nav_msgs/MapMetaData
    << "data"   << toBson(msg.data)   // int8[]
  << finalize;
}

value toBson(const nav_msgs::msg::Odometry& msg) {
  return document{}
    << "header"         << toBson(msg.header)         // std_msgs/Header
    << "child_frame_id" << toBson(msg.child_frame_id) // string
    << "pose"           << toBson(msg.pose)           // geometry_msgs/PoseWithCovariance
    << "twist"          << toBson(msg.twist)          // geometry_msgs/TwistWithCovariance
  << finalize;
}

value toBson(const nav_msgs::msg::Path& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "poses"  << toBson(msg.poses)  // geometry_msgs/PoseStamped[]
  << finalize;
}

value toBson(const perception_msgs::msg::EgoData& msg) {
  return document{}
    << "header"             << toBson(msg.header)             // std_msgs/Header
    << "state"              << toBson(msg.state)              // perception_msgs/ObjectState
    << "trajectory_planned" << toBson(msg.trajectory_planned) // perception_msgs/ObjectState[]
    << "trajectory_past"    << toBson(msg.trajectory_past)    // perception_msgs/ObjectState[]
    << "route_planned"      << toBson(msg.route_planned)      // geometry_msgs/Point[]
    << "vehicle_id"         << toBson(msg.vehicle_id)         // uint64
    << "length"             << toBson(msg.length)             // float64
    << "width"              << toBson(msg.width)              // float64
    << "height"             << toBson(msg.height)             // float64
  << finalize;
}

value toBson(const perception_msgs::msg::Object& msg) {
  return document{}
    << "id"                    << toBson(msg.id)                    // uint64
    << "existence_probability" << toBson(msg.existence_probability) // float64
    << "state"                 << toBson(msg.state)                 // perception_msgs/ObjectState
    << "state_history"         << toBson(msg.state_history)         // perception_msgs/ObjectState[]
    << "state_predictions"     << toBson(msg.state_predictions)     // perception_msgs/ObjectStatePrediction[]
  << finalize;
}

value toBson(const perception_msgs::msg::ObjectClassification& msg) {
  return document{}
    << "type"        << toBson(msg.type)        // uint8
    << "probability" << toBson(msg.probability) // float64
  << finalize;
}

value toBson(const perception_msgs::msg::ObjectList& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "objects" << toBson(msg.objects) // perception_msgs/Object[]
  << finalize;
}

value toBson(const perception_msgs::msg::ObjectReferencePoint& msg) {
  return document{}
    << "value"                           << toBson(msg.value)                           // uint8
    << "translation_to_geometric_center" << toBson(msg.translation_to_geometric_center) // geometry_msgs/Vector3
  << finalize;
}

value toBson(const perception_msgs::msg::ObjectState& msg) {
  return document{}
    << "header"                      << toBson(msg.header)                      // std_msgs/Header
    << "model_id"                    << toBson(msg.model_id)                    // uint8
    << "sensor_id"                   << toBson(msg.sensor_id)                   // uint64[]
    << "continuous_state"            << toBson(msg.continuous_state)            // float64[]
    << "discrete_state"              << toBson(msg.discrete_state)              // int64[]
    << "continuous_state_covariance" << toBson(msg.continuous_state_covariance) // float64[]
    << "classifications"             << toBson(msg.classifications)             // perception_msgs/ObjectClassification[]
    << "reference_point"             << toBson(msg.reference_point)             // perception_msgs/ObjectReferencePoint
  << finalize;
}

value toBson(const perception_msgs::msg::ObjectStatePrediction& msg) {
  return document{}
    << "probability" << toBson(msg.probability) // float64
    << "states"      << toBson(msg.states)      // perception_msgs/ObjectState[]
  << finalize;
}

value toBson(const sensor_msgs::msg::BatteryState& msg) {
  return document{}
    << "header"                  << toBson(msg.header)                  // std_msgs/Header
    << "voltage"                 << toBson(msg.voltage)                 // float32
    << "temperature"             << toBson(msg.temperature)             // float32
    << "current"                 << toBson(msg.current)                 // float32
    << "charge"                  << toBson(msg.charge)                  // float32
    << "capacity"                << toBson(msg.capacity)                // float32
    << "design_capacity"         << toBson(msg.design_capacity)         // float32
    << "percentage"              << toBson(msg.percentage)              // float32
    << "power_supply_status"     << toBson(msg.power_supply_status)     // uint8
    << "power_supply_health"     << toBson(msg.power_supply_health)     // uint8
    << "power_supply_technology" << toBson(msg.power_supply_technology) // uint8
    << "present"                 << toBson(msg.present)                 // bool
    << "cell_voltage"            << toBson(msg.cell_voltage)            // float32[]
    << "cell_temperature"        << toBson(msg.cell_temperature)        // float32[]
    << "location"                << toBson(msg.location)                // string
    << "serial_number"           << toBson(msg.serial_number)           // string
  << finalize;
}

value toBson(const sensor_msgs::msg::CameraInfo& msg) {
  return document{}
    << "header"           << toBson(msg.header)           // std_msgs/Header
    << "height"           << toBson(msg.height)           // uint32
    << "width"            << toBson(msg.width)            // uint32
    << "distortion_model" << toBson(msg.distortion_model) // string
    << "d"                << toBson(msg.d)                // float64[]
    << "k"                << toBson(msg.k)                // float64[9]
    << "r"                << toBson(msg.r)                // float64[9]
    << "p"                << toBson(msg.p)                // float64[12]
    << "binning_x"        << toBson(msg.binning_x)        // uint32
    << "binning_y"        << toBson(msg.binning_y)        // uint32
    << "roi"              << toBson(msg.roi)              // sensor_msgs/RegionOfInterest
  << finalize;
}

value toBson(const sensor_msgs::msg::ChannelFloat32& msg) {
  return document{}
    << "name"   << toBson(msg.name)   // string
    << "values" << toBson(msg.values) // float32[]
  << finalize;
}

value toBson(const sensor_msgs::msg::CompressedImage& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "format" << toBson(msg.format) // string
    << "data"   << toBson(msg.data)   // uint8[]
  << finalize;
}

value toBson(const sensor_msgs::msg::FluidPressure& msg) {
  return document{}
    << "header"         << toBson(msg.header)         // std_msgs/Header
    << "fluid_pressure" << toBson(msg.fluid_pressure) // float64
    << "variance"       << toBson(msg.variance)       // float64
  << finalize;
}

value toBson(const sensor_msgs::msg::Illuminance& msg) {
  return document{}
    << "header"      << toBson(msg.header)      // std_msgs/Header
    << "illuminance" << toBson(msg.illuminance) // float64
    << "variance"    << toBson(msg.variance)    // float64
  << finalize;
}

value toBson(const sensor_msgs::msg::Image& msg) {
  return document{}
    << "header"       << toBson(msg.header)       // std_msgs/Header
    << "height"       << toBson(msg.height)       // uint32
    << "width"        << toBson(msg.width)        // uint32
    << "encoding"     << toBson(msg.encoding)     // string
    << "is_bigendian" << toBson(msg.is_bigendian) // uint8
    << "step"         << toBson(msg.step)         // uint32
    << "data"         << toBson(msg.data)         // uint8[]
  << finalize;
}

value toBson(const sensor_msgs::msg::Imu& msg) {
  return document{}
    << "header"                         << toBson(msg.header)                         // std_msgs/Header
    << "orientation"                    << toBson(msg.orientation)                    // geometry_msgs/Quaternion
    << "orientation_covariance"         << toBson(msg.orientation_covariance)         // float64[9]
    << "angular_velocity"               << toBson(msg.angular_velocity)               // geometry_msgs/Vector3
    << "angular_velocity_covariance"    << toBson(msg.angular_velocity_covariance)    // float64[9]
    << "linear_acceleration"            << toBson(msg.linear_acceleration)            // geometry_msgs/Vector3
    << "linear_acceleration_covariance" << toBson(msg.linear_acceleration_covariance) // float64[9]
  << finalize;
}

value toBson(const sensor_msgs::msg::JointState& msg) {
  return document{}
    << "header"   << toBson(msg.header)   // std_msgs/Header
    << "name"     << toBson(msg.name)     // string[]
    << "position" << toBson(msg.position) // float64[]
    << "velocity" << toBson(msg.velocity) // float64[]
    << "effort"   << toBson(msg.effort)   // float64[]
  << finalize;
}

value toBson(const sensor_msgs::msg::Joy& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "axes"    << toBson(msg.axes)    // float32[]
    << "buttons" << toBson(msg.buttons) // int32[]
  << finalize;
}

value toBson(const sensor_msgs::msg::JoyFeedback& msg) {
  return document{}
    << "type"      << toBson(msg.type)      // uint8
    << "id"        << toBson(msg.id)        // uint8
    << "intensity" << toBson(msg.intensity) // float32
  << finalize;
}

value toBson(const sensor_msgs::msg::JoyFeedbackArray& msg) {
  return document{}
    << "array" << toBson(msg.array) // sensor_msgs/JoyFeedback[]
  << finalize;
}

value toBson(const sensor_msgs::msg::LaserEcho& msg) {
  return document{}
    << "echoes" << toBson(msg.echoes) // float32[]
  << finalize;
}

value toBson(const sensor_msgs::msg::LaserScan& msg) {
  return document{}
    << "header"          << toBson(msg.header)          // std_msgs/Header
    << "angle_min"       << toBson(msg.angle_min)       // float32
    << "angle_max"       << toBson(msg.angle_max)       // float32
    << "angle_increment" << toBson(msg.angle_increment) // float32
    << "time_increment"  << toBson(msg.time_increment)  // float32
    << "scan_time"       << toBson(msg.scan_time)       // float32
    << "range_min"       << toBson(msg.range_min)       // float32
    << "range_max"       << toBson(msg.range_max)       // float32
    << "ranges"          << toBson(msg.ranges)          // float32[]
    << "intensities"     << toBson(msg.intensities)     // float32[]
  << finalize;
}

value toBson(const sensor_msgs::msg::MagneticField& msg) {
  return document{}
    << "header"                    << toBson(msg.header)                    // std_msgs/Header
    << "magnetic_field"            << toBson(msg.magnetic_field)            // geometry_msgs/Vector3
    << "magnetic_field_covariance" << toBson(msg.magnetic_field_covariance) // float64[9]
  << finalize;
}

value toBson(const sensor_msgs::msg::MultiDOFJointState& msg) {
  return document{}
    << "header"      << toBson(msg.header)      // std_msgs/Header
    << "joint_names" << toBson(msg.joint_names) // string[]
    << "transforms"  << toBson(msg.transforms)  // geometry_msgs/Transform[]
    << "twist"       << toBson(msg.twist)       // geometry_msgs/Twist[]
    << "wrench"      << toBson(msg.wrench)      // geometry_msgs/Wrench[]
  << finalize;
}

value toBson(const sensor_msgs::msg::MultiEchoLaserScan& msg) {
  return document{}
    << "header"          << toBson(msg.header)          // std_msgs/Header
    << "angle_min"       << toBson(msg.angle_min)       // float32
    << "angle_max"       << toBson(msg.angle_max)       // float32
    << "angle_increment" << toBson(msg.angle_increment) // float32
    << "time_increment"  << toBson(msg.time_increment)  // float32
    << "scan_time"       << toBson(msg.scan_time)       // float32
    << "range_min"       << toBson(msg.range_min)       // float32
    << "range_max"       << toBson(msg.range_max)       // float32
    << "ranges"          << toBson(msg.ranges)          // sensor_msgs/LaserEcho[]
    << "intensities"     << toBson(msg.intensities)     // sensor_msgs/LaserEcho[]
  << finalize;
}

value toBson(const sensor_msgs::msg::NavSatFix& msg) {
  return document{}
    << "header"                   << toBson(msg.header)                   // std_msgs/Header
    << "status"                   << toBson(msg.status)                   // sensor_msgs/NavSatStatus
    << "latitude"                 << toBson(msg.latitude)                 // float64
    << "longitude"                << toBson(msg.longitude)                // float64
    << "altitude"                 << toBson(msg.altitude)                 // float64
    << "position_covariance"      << toBson(msg.position_covariance)      // float64[9]
    << "position_covariance_type" << toBson(msg.position_covariance_type) // uint8
  << finalize;
}

value toBson(const sensor_msgs::msg::NavSatStatus& msg) {
  return document{}
    << "status"  << toBson(msg.status)  // int8
    << "service" << toBson(msg.service) // uint16
  << finalize;
}

value toBson(const sensor_msgs::msg::PointCloud& msg) {
  return document{}
    << "header"   << toBson(msg.header)   // std_msgs/Header
    << "points"   << toBson(msg.points)   // geometry_msgs/Point32[]
    << "channels" << toBson(msg.channels) // sensor_msgs/ChannelFloat32[]
  << finalize;
}

value toBson(const sensor_msgs::msg::PointCloud2& msg) {
  return document{}
    << "header"       << toBson(msg.header)       // std_msgs/Header
    << "height"       << toBson(msg.height)       // uint32
    << "width"        << toBson(msg.width)        // uint32
    << "fields"       << toBson(msg.fields)       // sensor_msgs/PointField[]
    << "is_bigendian" << toBson(msg.is_bigendian) // bool
    << "point_step"   << toBson(msg.point_step)   // uint32
    << "row_step"     << toBson(msg.row_step)     // uint32
    << "data"         << toBson(msg.data)         // uint8[]
    << "is_dense"     << toBson(msg.is_dense)     // bool
  << finalize;
}

value toBson(const sensor_msgs::msg::PointField& msg) {
  return document{}
    << "name"     << toBson(msg.name)     // string
    << "offset"   << toBson(msg.offset)   // uint32
    << "datatype" << toBson(msg.datatype) // uint8
    << "count"    << toBson(msg.count)    // uint32
  << finalize;
}

value toBson(const sensor_msgs::msg::Range& msg) {
  return document{}
    << "header"         << toBson(msg.header)         // std_msgs/Header
    << "radiation_type" << toBson(msg.radiation_type) // uint8
    << "field_of_view"  << toBson(msg.field_of_view)  // float32
    << "min_range"      << toBson(msg.min_range)      // float32
    << "max_range"      << toBson(msg.max_range)      // float32
    << "range"          << toBson(msg.range)          // float32
  << finalize;
}

value toBson(const sensor_msgs::msg::RegionOfInterest& msg) {
  return document{}
    << "x_offset"   << toBson(msg.x_offset)   // uint32
    << "y_offset"   << toBson(msg.y_offset)   // uint32
    << "height"     << toBson(msg.height)     // uint32
    << "width"      << toBson(msg.width)      // uint32
    << "do_rectify" << toBson(msg.do_rectify) // bool
  << finalize;
}

value toBson(const sensor_msgs::msg::RelativeHumidity& msg) {
  return document{}
    << "header"            << toBson(msg.header)            // std_msgs/Header
    << "relative_humidity" << toBson(msg.relative_humidity) // float64
    << "variance"          << toBson(msg.variance)          // float64
  << finalize;
}

value toBson(const sensor_msgs::msg::Temperature& msg) {
  return document{}
    << "header"      << toBson(msg.header)      // std_msgs/Header
    << "temperature" << toBson(msg.temperature) // float64
    << "variance"    << toBson(msg.variance)    // float64
  << finalize;
}

value toBson(const sensor_msgs::msg::TimeReference& msg) {
  return document{}
    << "header"   << toBson(msg.header)   // std_msgs/Header
    << "time_ref" << toBson(msg.time_ref) // builtin_interfaces/Time
    << "source"   << toBson(msg.source)   // string
  << finalize;
}

value toBson(const shape_msgs::msg::Mesh& msg) {
  return document{}
    << "triangles" << toBson(msg.triangles) // shape_msgs/MeshTriangle[]
    << "vertices"  << toBson(msg.vertices)  // geometry_msgs/Point[]
  << finalize;
}

value toBson(const shape_msgs::msg::MeshTriangle& msg) {
  return document{}
    << "vertex_indices" << toBson(msg.vertex_indices) // uint32[3]
  << finalize;
}

value toBson(const shape_msgs::msg::Plane& msg) {
  return document{}
    << "coef" << toBson(msg.coef) // float64[4]
  << finalize;
}

value toBson(const shape_msgs::msg::SolidPrimitive& msg) {
  return document{}
    << "type"    << toBson(msg.type)    // uint8
    << "polygon" << toBson(msg.polygon) // geometry_msgs/Polygon
  << finalize;
}

value toBson(const std_msgs::msg::Bool& msg) {
  return document{}
    << "data" << toBson(msg.data) // bool
  << finalize;
}

value toBson(const std_msgs::msg::Byte& msg) {
  return document{}
    << "data" << toBson(msg.data) // byte
  << finalize;
}

value toBson(const std_msgs::msg::ByteMultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // byte[]
  << finalize;
}

value toBson(const std_msgs::msg::Char& msg) {
  return document{}
    << "data" << toBson(msg.data) // char
  << finalize;
}

value toBson(const std_msgs::msg::ColorRGBA& msg) {
  return document{}
    << "r" << toBson(msg.r) // float32
    << "g" << toBson(msg.g) // float32
    << "b" << toBson(msg.b) // float32
    << "a" << toBson(msg.a) // float32
  << finalize;
}

value toBson(const std_msgs::msg::Empty& msg) {
  return document{}
  << finalize;
}

value toBson(const std_msgs::msg::Float32& msg) {
  return document{}
    << "data" << toBson(msg.data) // float32
  << finalize;
}

value toBson(const std_msgs::msg::Float32MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // float32[]
  << finalize;
}

value toBson(const std_msgs::msg::Float64& msg) {
  return document{}
    << "data" << toBson(msg.data) // float64
  << finalize;
}

value toBson(const std_msgs::msg::Float64MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // float64[]
  << finalize;
}

value toBson(const std_msgs::msg::Header& msg) {
  return document{}
    << "stamp"    << toBson(msg.stamp)    // builtin_interfaces/Time
    << "frame_id" << toBson(msg.frame_id) // string
  << finalize;
}

value toBson(const std_msgs::msg::Int16& msg) {
  return document{}
    << "data" << toBson(msg.data) // int16
  << finalize;
}

value toBson(const std_msgs::msg::Int16MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // int16[]
  << finalize;
}

value toBson(const std_msgs::msg::Int32& msg) {
  return document{}
    << "data" << toBson(msg.data) // int32
  << finalize;
}

value toBson(const std_msgs::msg::Int32MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // int32[]
  << finalize;
}

value toBson(const std_msgs::msg::Int64& msg) {
  return document{}
    << "data" << toBson(msg.data) // int64
  << finalize;
}

value toBson(const std_msgs::msg::Int64MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // int64[]
  << finalize;
}

value toBson(const std_msgs::msg::Int8& msg) {
  return document{}
    << "data" << toBson(msg.data) // int8
  << finalize;
}

value toBson(const std_msgs::msg::Int8MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // int8[]
  << finalize;
}

value toBson(const std_msgs::msg::MultiArrayDimension& msg) {
  return document{}
    << "label"  << toBson(msg.label)  // string
    << "size"   << toBson(msg.size)   // uint32
    << "stride" << toBson(msg.stride) // uint32
  << finalize;
}

value toBson(const std_msgs::msg::MultiArrayLayout& msg) {
  return document{}
    << "dim"         << toBson(msg.dim)         // std_msgs/MultiArrayDimension[]
    << "data_offset" << toBson(msg.data_offset) // uint32
  << finalize;
}

value toBson(const std_msgs::msg::String& msg) {
  return document{}
    << "data" << toBson(msg.data) // string
  << finalize;
}

value toBson(const std_msgs::msg::UInt16& msg) {
  return document{}
    << "data" << toBson(msg.data) // uint16
  << finalize;
}

value toBson(const std_msgs::msg::UInt16MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // uint16[]
  << finalize;
}

value toBson(const std_msgs::msg::UInt32& msg) {
  return document{}
    << "data" << toBson(msg.data) // uint32
  << finalize;
}

value toBson(const std_msgs::msg::UInt32MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // uint32[]
  << finalize;
}

value toBson(const std_msgs::msg::UInt64& msg) {
  return document{}
    << "data" << toBson(msg.data) // uint64
  << finalize;
}

value toBson(const std_msgs::msg::UInt64MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // uint64[]
  << finalize;
}

value toBson(const std_msgs::msg::UInt8& msg) {
  return document{}
    << "data" << toBson(msg.data) // uint8
  << finalize;
}

value toBson(const std_msgs::msg::UInt8MultiArray& msg) {
  return document{}
    << "layout" << toBson(msg.layout) // std_msgs/MultiArrayLayout
    << "data"   << toBson(msg.data)   // uint8[]
  << finalize;
}

value toBson(const stereo_msgs::msg::DisparityImage& msg) {
  return document{}
    << "header"        << toBson(msg.header)        // std_msgs/Header
    << "image"         << toBson(msg.image)         // sensor_msgs/Image
    << "f"             << toBson(msg.f)             // float32
    << "t"             << toBson(msg.t)             // float32
    << "valid_window"  << toBson(msg.valid_window)  // sensor_msgs/RegionOfInterest
    << "min_disparity" << toBson(msg.min_disparity) // float32
    << "max_disparity" << toBson(msg.max_disparity) // float32
    << "delta_d"       << toBson(msg.delta_d)       // float32
  << finalize;
}

value toBson(const tf2_msgs::msg::TF2Error& msg) {
  return document{}
    << "error"        << toBson(msg.error)        // uint8
    << "error_string" << toBson(msg.error_string) // string
  << finalize;
}

value toBson(const tf2_msgs::msg::TFMessage& msg) {
  return document{}
    << "transforms" << toBson(msg.transforms) // geometry_msgs/TransformStamped[]
  << finalize;
}

value toBson(const trajectory_msgs::msg::JointTrajectory& msg) {
  return document{}
    << "header"      << toBson(msg.header)      // std_msgs/Header
    << "joint_names" << toBson(msg.joint_names) // string[]
    << "points"      << toBson(msg.points)      // trajectory_msgs/JointTrajectoryPoint[]
  << finalize;
}

value toBson(const trajectory_msgs::msg::JointTrajectoryPoint& msg) {
  return document{}
    << "positions"       << toBson(msg.positions)       // float64[]
    << "velocities"      << toBson(msg.velocities)      // float64[]
    << "accelerations"   << toBson(msg.accelerations)   // float64[]
    << "effort"          << toBson(msg.effort)          // float64[]
    << "time_from_start" << toBson(msg.time_from_start) // builtin_interfaces/Duration
  << finalize;
}

value toBson(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg) {
  return document{}
    << "header"      << toBson(msg.header)      // std_msgs/Header
    << "joint_names" << toBson(msg.joint_names) // string[]
    << "points"      << toBson(msg.points)      // trajectory_msgs/MultiDOFJointTrajectoryPoint[]
  << finalize;
}

value toBson(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg) {
  return document{}
    << "transforms"      << toBson(msg.transforms)      // geometry_msgs/Transform[]
    << "velocities"      << toBson(msg.velocities)      // geometry_msgs/Twist[]
    << "accelerations"   << toBson(msg.accelerations)   // geometry_msgs/Twist[]
    << "time_from_start" << toBson(msg.time_from_start) // builtin_interfaces/Duration
  << finalize;
}

value toBson(const vision_msgs::msg::BoundingBox2D& msg) {
  return document{}
    << "center" << toBson(msg.center) // vision_msgs/Pose2D
    << "size_x" << toBson(msg.size_x) // float64
    << "size_y" << toBson(msg.size_y) // float64
  << finalize;
}

value toBson(const vision_msgs::msg::BoundingBox2DArray& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "boxes"  << toBson(msg.boxes)  // vision_msgs/BoundingBox2D[]
  << finalize;
}

value toBson(const vision_msgs::msg::BoundingBox3D& msg) {
  return document{}
    << "center" << toBson(msg.center) // geometry_msgs/Pose
    << "size"   << toBson(msg.size)   // geometry_msgs/Vector3
  << finalize;
}

value toBson(const vision_msgs::msg::BoundingBox3DArray& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "boxes"  << toBson(msg.boxes)  // vision_msgs/BoundingBox3D[]
  << finalize;
}

value toBson(const vision_msgs::msg::Classification& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "results" << toBson(msg.results) // vision_msgs/ObjectHypothesis[]
  << finalize;
}

value toBson(const vision_msgs::msg::Detection2D& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "results" << toBson(msg.results) // vision_msgs/ObjectHypothesisWithPose[]
    << "bbox"    << toBson(msg.bbox)    // vision_msgs/BoundingBox2D
    << "id"      << toBson(msg.id)      // string
  << finalize;
}

value toBson(const vision_msgs::msg::Detection2DArray& msg) {
  return document{}
    << "header"     << toBson(msg.header)     // std_msgs/Header
    << "detections" << toBson(msg.detections) // vision_msgs/Detection2D[]
  << finalize;
}

value toBson(const vision_msgs::msg::Detection3D& msg) {
  return document{}
    << "header"  << toBson(msg.header)  // std_msgs/Header
    << "results" << toBson(msg.results) // vision_msgs/ObjectHypothesisWithPose[]
    << "bbox"    << toBson(msg.bbox)    // vision_msgs/BoundingBox3D
    << "id"      << toBson(msg.id)      // string
  << finalize;
}

value toBson(const vision_msgs::msg::Detection3DArray& msg) {
  return document{}
    << "header"     << toBson(msg.header)     // std_msgs/Header
    << "detections" << toBson(msg.detections) // vision_msgs/Detection3D[]
  << finalize;
}

value toBson(const vision_msgs::msg::LabelInfo& msg) {
  return document{}
    << "header"    << toBson(msg.header)    // std_msgs/Header
    << "class_map" << toBson(msg.class_map) // vision_msgs/VisionClass[]
    << "threshold" << toBson(msg.threshold) // float32
  << finalize;
}

value toBson(const vision_msgs::msg::ObjectHypothesis& msg) {
  return document{}
    << "class_id" << toBson(msg.class_id) // string
    << "score"    << toBson(msg.score)    // float64
  << finalize;
}

value toBson(const vision_msgs::msg::ObjectHypothesisWithPose& msg) {
  return document{}
    << "hypothesis" << toBson(msg.hypothesis) // vision_msgs/ObjectHypothesis
    << "pose"       << toBson(msg.pose)       // geometry_msgs/PoseWithCovariance
  << finalize;
}

value toBson(const vision_msgs::msg::Point2D& msg) {
  return document{}
    << "x" << toBson(msg.x) // float64
    << "y" << toBson(msg.y) // float64
  << finalize;
}

value toBson(const vision_msgs::msg::Pose2D& msg) {
  return document{}
    << "position" << toBson(msg.position) // vision_msgs/Point2D
    << "theta"    << toBson(msg.theta)    // float64
  << finalize;
}

value toBson(const vision_msgs::msg::VisionClass& msg) {
  return document{}
    << "class_id"   << toBson(msg.class_id)   // uint16
    << "class_name" << toBson(msg.class_name) // string
  << finalize;
}

value toBson(const vision_msgs::msg::VisionInfo& msg) {
  return document{}
    << "header"            << toBson(msg.header)            // std_msgs/Header
    << "method"            << toBson(msg.method)            // string
    << "database_location" << toBson(msg.database_location) // string
    << "database_version"  << toBson(msg.database_version)  // int32
  << finalize;
}

value toBson(const visualization_msgs::msg::ImageMarker& msg) {
  return document{}
    << "header"         << toBson(msg.header)         // std_msgs/Header
    << "ns"             << toBson(msg.ns)             // string
    << "id"             << toBson(msg.id)             // int32
    << "type"           << toBson(msg.type)           // int32
    << "action"         << toBson(msg.action)         // int32
    << "position"       << toBson(msg.position)       // geometry_msgs/Point
    << "scale"          << toBson(msg.scale)          // float32
    << "outline_color"  << toBson(msg.outline_color)  // std_msgs/ColorRGBA
    << "filled"         << toBson(msg.filled)         // uint8
    << "fill_color"     << toBson(msg.fill_color)     // std_msgs/ColorRGBA
    << "lifetime"       << toBson(msg.lifetime)       // builtin_interfaces/Duration
    << "points"         << toBson(msg.points)         // geometry_msgs/Point[]
    << "outline_colors" << toBson(msg.outline_colors) // std_msgs/ColorRGBA[]
  << finalize;
}

value toBson(const visualization_msgs::msg::InteractiveMarker& msg) {
  return document{}
    << "header"       << toBson(msg.header)       // std_msgs/Header
    << "pose"         << toBson(msg.pose)         // geometry_msgs/Pose
    << "name"         << toBson(msg.name)         // string
    << "description"  << toBson(msg.description)  // string
    << "scale"        << toBson(msg.scale)        // float32
    << "menu_entries" << toBson(msg.menu_entries) // visualization_msgs/MenuEntry[]
    << "controls"     << toBson(msg.controls)     // visualization_msgs/InteractiveMarkerControl[]
  << finalize;
}

value toBson(const visualization_msgs::msg::InteractiveMarkerControl& msg) {
  return document{}
    << "name"                           << toBson(msg.name)                           // string
    << "orientation"                    << toBson(msg.orientation)                    // geometry_msgs/Quaternion
    << "orientation_mode"               << toBson(msg.orientation_mode)               // uint8
    << "interaction_mode"               << toBson(msg.interaction_mode)               // uint8
    << "always_visible"                 << toBson(msg.always_visible)                 // bool
    << "markers"                        << toBson(msg.markers)                        // visualization_msgs/Marker[]
    << "independent_marker_orientation" << toBson(msg.independent_marker_orientation) // bool
    << "description"                    << toBson(msg.description)                    // string
  << finalize;
}

value toBson(const visualization_msgs::msg::InteractiveMarkerFeedback& msg) {
  return document{}
    << "header"            << toBson(msg.header)            // std_msgs/Header
    << "client_id"         << toBson(msg.client_id)         // string
    << "marker_name"       << toBson(msg.marker_name)       // string
    << "control_name"      << toBson(msg.control_name)      // string
    << "event_type"        << toBson(msg.event_type)        // uint8
    << "pose"              << toBson(msg.pose)              // geometry_msgs/Pose
    << "menu_entry_id"     << toBson(msg.menu_entry_id)     // uint32
    << "mouse_point"       << toBson(msg.mouse_point)       // geometry_msgs/Point
    << "mouse_point_valid" << toBson(msg.mouse_point_valid) // bool
  << finalize;
}

value toBson(const visualization_msgs::msg::InteractiveMarkerInit& msg) {
  return document{}
    << "server_id" << toBson(msg.server_id) // string
    << "seq_num"   << toBson(msg.seq_num)   // uint64
    << "markers"   << toBson(msg.markers)   // visualization_msgs/InteractiveMarker[]
  << finalize;
}

value toBson(const visualization_msgs::msg::InteractiveMarkerPose& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "pose"   << toBson(msg.pose)   // geometry_msgs/Pose
    << "name"   << toBson(msg.name)   // string
  << finalize;
}

value toBson(const visualization_msgs::msg::InteractiveMarkerUpdate& msg) {
  return document{}
    << "server_id" << toBson(msg.server_id) // string
    << "seq_num"   << toBson(msg.seq_num)   // uint64
    << "type"      << toBson(msg.type)      // uint8
    << "markers"   << toBson(msg.markers)   // visualization_msgs/InteractiveMarker[]
    << "poses"     << toBson(msg.poses)     // visualization_msgs/InteractiveMarkerPose[]
    << "erases"    << toBson(msg.erases)    // string[]
  << finalize;
}

value toBson(const visualization_msgs::msg::Marker& msg) {
  return document{}
    << "header"                      << toBson(msg.header)                      // std_msgs/Header
    << "ns"                          << toBson(msg.ns)                          // string
    << "id"                          << toBson(msg.id)                          // int32
    << "type"                        << toBson(msg.type)                        // int32
    << "action"                      << toBson(msg.action)                      // int32
    << "pose"                        << toBson(msg.pose)                        // geometry_msgs/Pose
    << "scale"                       << toBson(msg.scale)                       // geometry_msgs/Vector3
    << "color"                       << toBson(msg.color)                       // std_msgs/ColorRGBA
    << "lifetime"                    << toBson(msg.lifetime)                    // builtin_interfaces/Duration
    << "frame_locked"                << toBson(msg.frame_locked)                // bool
    << "points"                      << toBson(msg.points)                      // geometry_msgs/Point[]
    << "colors"                      << toBson(msg.colors)                      // std_msgs/ColorRGBA[]
    << "texture_resource"            << toBson(msg.texture_resource)            // string
    << "texture"                     << toBson(msg.texture)                     // sensor_msgs/CompressedImage
    << "uv_coordinates"              << toBson(msg.uv_coordinates)              // visualization_msgs/UVCoordinate[]
    << "text"                        << toBson(msg.text)                        // string
    << "mesh_resource"               << toBson(msg.mesh_resource)               // string
    << "mesh_file"                   << toBson(msg.mesh_file)                   // visualization_msgs/MeshFile
    << "mesh_use_embedded_materials" << toBson(msg.mesh_use_embedded_materials) // bool
  << finalize;
}

value toBson(const visualization_msgs::msg::MarkerArray& msg) {
  return document{}
    << "markers" << toBson(msg.markers) // visualization_msgs/Marker[]
  << finalize;
}

value toBson(const visualization_msgs::msg::MenuEntry& msg) {
  return document{}
    << "id"           << toBson(msg.id)           // uint32
    << "parent_id"    << toBson(msg.parent_id)    // uint32
    << "title"        << toBson(msg.title)        // string
    << "command"      << toBson(msg.command)      // string
    << "command_type" << toBson(msg.command_type) // uint8
  << finalize;
}

value toBson(const visualization_msgs::msg::MeshFile& msg) {
  return document{}
    << "filename" << toBson(msg.filename) // string
    << "data"     << toBson(msg.data)     // uint8[]
  << finalize;
}

value toBson(const visualization_msgs::msg::UVCoordinate& msg) {
  return document{}
    << "u" << toBson(msg.u) // float32
    << "v" << toBson(msg.v) // float32
  << finalize;
}

value toBson(const event_detector_db_recording_msgs::msg::ImageFile& msg) {
  return document{}
    << "header" << toBson(msg.header) // std_msgs/Header
    << "height" << toBson(msg.height) // uint32
    << "width"  << toBson(msg.width)  // uint32
    << "file"   << toBson(msg.file)   // string
  << finalize;
}

value toBson(const event_detector_db_recording_msgs::msg::PointCloudFile& msg) {
  return document{}
    << "header"     << toBson(msg.header)     // std_msgs/Header
    << "num_points" << toBson(msg.num_points) // uint32
    << "file"       << toBson(msg.file)       // string
  << finalize;
}

// clang-format on
}  // namespace event_detector_db_recording_plugin
