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

#include <unordered_set>
#include <vector>

#include <event_detector/EventDetector.hpp>

#include <pluginlib/class_list_macros.hpp>
#include "event_detector_db_recording_plugin/rules/record_all/RecordAllRule.hpp"
PLUGINLIB_EXPORT_CLASS(event_detector_db_recording_plugin::RecordAllRule, event_detector::AnalysisRule)

namespace event_detector_db_recording_plugin {

std::string RecordAllRule::getRuleName() const { return "event_detector_db_recording_plugin::RecordAllRule"; }

void RecordAllRule::onInitialize() { start_time_ = ed_->now(); }

void RecordAllRule::loadRuleParameters() {
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION)                              \
  this->loadRuleParameter<bool>("" #VAR, VAR##_, false);                         \
  if (VAR##_) {                                                                  \
    RCLCPP_INFO(rclcpp::get_logger(this->getRuleName()), "Recording " #VAR);     \
  } else {                                                                       \
    RCLCPP_INFO(rclcpp::get_logger(this->getRuleName()), "Not recording " #VAR); \
  }
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE

  // load generic parameters
  this->RecordingRule::loadRuleParameters();
}

void RecordAllRule::evaluate() {
  EvaluationResult result;
  ScenarioDataRequest s_request;
  s_request.scenario_types = scenario_types_;

  // create client requests based on available clients per selected datatype
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION)                            \
  if (VAR##_) {                                                                \
    std::vector<std::string> VAR##_clients = buffer_->VAR.getClients();        \
    for (const std::string& client : VAR##_clients) {                          \
      if (s_request.client_requests.count(client) == 0) {                      \
        s_request.client_requests[client] = ClientDataRequest();               \
        s_request.client_requests[client].tf2_msgs__TFMessage.request = false; \
      }                                                                        \
      s_request.client_requests[client].VAR.request = true;                    \
    }                                                                          \
  }
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE

  // set current time window to record
  rclcpp::Time end_time = ed_->now();
  for (auto& v_request : s_request.client_requests) {
    v_request.second.start_time = start_time_;
    v_request.second.end_time = end_time;
  }

  // log info
  if (s_request.client_requests.size() > 0) {
    result.scenario_requests.push_back(s_request);
    RCLCPP_INFO(rclcpp::get_logger(this->getRuleName()), "Recording %.1fs of selected data from %ld clients",
                (end_time - start_time_).seconds(), s_request.client_requests.size());
  }

  // store current end time as starting point for next storage period
  start_time_ = end_time;

  this->trigger(result);
}

}  // namespace event_detector_db_recording_plugin
