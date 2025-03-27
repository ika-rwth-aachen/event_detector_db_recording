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

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <event_detector/common.hpp>
#include <rclcpp/rclcpp.hpp>

#include "event_detector_db_recording_plugin/datatypes.hpp"

namespace event_detector_db_recording_plugin {

namespace ed = event_detector;

const std::string kClientBaseFrame = "base_link";        ///< standard name of base ROS frame
const std::string kClientGeoCenterFrame = "geo_center";  ///< standard name of geometric client center ROS frame
const std::set<std::string> kNonClientFrames = {
    "map"};  ///< ROS frames that in general are not rigidly attached to a client

/**
 * @brief Struct representing a request for a specific data type
 */
struct DataRequest {
  bool request = false;          ///< whether this data is requested
  int n_samples = -1;            ///< maximum number of samples to request
  std::set<std::string> frames;  ///< requested frames, may be empty
};

/**
 * @brief Struct representing a data request per client resulting from rule evaluation
 */
struct ClientDataRequest {
  rclcpp::Time start_time;                                            ///< request data after this time
  rclcpp::Time end_time;                                              ///< request data before this time
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION) DataRequest VAR;  ///< TYPE request information
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE
  // request transforms by default
  ClientDataRequest() : tf2_msgs__TFMessage(DataRequest{true, -1, {}}) {}
};

/**
 * @brief Struct representing a data request for a scenario resulting from rule
 * evaluation
 */
struct ScenarioDataRequest {
  std::set<std::string> rules;                                         ///< names of responsible analysis rules
  std::set<std::string> scenario_types;                                ///< list of identified ScenarioTypes
  std::string metadata = "{}";                                         ///< json-string representing arbitrary metadata
  std::unordered_map<std::string, ClientDataRequest> client_requests;  ///< ClientDataRequest per client id
};

/**
 * @brief Struct representing the result of a rule evaluation
 */
struct EvaluationResult {
  std::vector<ScenarioDataRequest> scenario_requests;  ///< requests for individual scenarios
};

/**
 * @brief Struct containing all data belonging to a scenario
 */
struct ScenarioData {
  std::set<std::string> rules;           ///< names of responsible analysis rules
  std::set<std::string> scenario_types;  ///< list of identified ScenarioTypes
  std::string metadata = "{}";           ///< json-string representing arbitrary metadata
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION) \
  ed::string_map_2d<std::vector<ed::Stamped<TYPE>>> VAR;  ///< TYPE by client ID by topic
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE
};

}  // namespace event_detector_db_recording_plugin