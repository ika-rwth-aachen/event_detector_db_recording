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

#include <cmath>
#include <vector>

#include <event_detector/EventDetector.hpp>

#include <pluginlib/class_list_macros.hpp>
#include "event_detector_db_recording_plugin/rules/template/TemplateRule.hpp"
PLUGINLIB_EXPORT_CLASS(event_detector_db_recording_plugin::TemplateRule, event_detector::AnalysisRule)

namespace event_detector_db_recording_plugin {

std::string TemplateRule::getRuleName() const {
  /**
   * This function is supposed to return the name of the rule.
   * For uniqueness, the suggested name is event_detector_db_recording_plugin::<ClassName>.
   */
  return "event_detector_db_recording_plugin::TemplateRule";
}

void TemplateRule::onInitialize() {
  /**
   * This function is called when the rule is initialized.
   * Use it to initialize member variables or perform other kinds of rule initialization.
   * Start by initializing the base class.
   */

  // initialize base class
  this->RecordingRule::onInitialize();
}

void TemplateRule::loadRuleParameters() {
  /**
   * This function declares and loads rule-specific parameters.
   * Here, we load the parameters x, y, and radius.
   */

  this->loadRuleParameter<double>("x", x_, x_);
  this->loadRuleParameter<double>("y", y_, y_);
  this->loadRuleParameter<double>("radius", radius_, radius_);

  /**
   * At the end, we should make sure to also load generic parameters of the base class.
   */

  // load generic parameters
  this->RecordingRule::loadRuleParameters();
}

void TemplateRule::evaluate() {
  /**
   * This function's purpose is to evaluate whether currently buffered data is
   * considered relevant and should be stored. This template rule will check if
   * a client that is sharing point clouds is near a specific xy-location. If
   * such a client is found, its latest pose and point cloud will be stored.
   * Note that this function constructs an EvaluationResult containing
   * information about which data to request for long-term storage.
   */

  // create empty EvaluationResult
  EvaluationResult result;

  // fetch list of clients with point clouds in the buffer
  std::vector<std::string> lidar_clients = buffer_->sensor_msgs__PointCloud2.getClients();
  if (lidar_clients.empty()) return;

  // fetch latest pose from the first identified client
  std::vector<ed::Stamped<geometry_msgs::msg::PoseStamped>> stamped_pose = buffer_->geometry_msgs__PoseStamped.get(1, lidar_clients[0], "");
  if (stamped_pose.empty()) return;
  /**
   * Check out the different getter functions that are provided by the buffer.
   * Whether the buffers' stamps refer to the time of arrival or the ROS header stamps is
   * determined by ROS parameter 'buffer.use_msg_stamp':
   *    false -> time of arrival
   *    true  -> ROS message stamp if present else time of arrival
   */

  // compute current distance of client to origin of observed area
  double dx = stamped_pose[0].msg->pose.position.x - x_;
  double dy = stamped_pose[0].msg->pose.position.x - y_;
  double r = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

  // in this case, data is considered to be relevant when the client position
  // lies within a specified radius (the observed area)
  if (r <= radius_) {
    // create a ClientDataRequest for the client including information about
    // what types of data from what time frame to request for long-term storage
    ClientDataRequest v_request;
    v_request.start_time = stamped_pose[0].msg->header.stamp;
    v_request.end_time = stamped_pose[0].msg->header.stamp;
    // v_request.tf2_msgs__TFMessage.request = true; // transforms requested by default
    v_request.geometry_msgs__PoseStamped.request = true;
    v_request.geometry_msgs__PoseStamped.n_samples = 1;
    v_request.sensor_msgs__PointCloud2.request = true;
    v_request.sensor_msgs__PointCloud2.n_samples = 1;
    // Note that what the timeframe refers to depends on the buffer
    // configuration (see comment above)

    // move the ClientDataRequest to an encompassing ScenarioDataRequest (which
    // may contain requests for multiple clients)
    ScenarioDataRequest s_request;
    s_request.client_requests[lidar_clients[0]] = v_request;

    // assign scenario types and optionally add arbitrarily nested JSON metadata
    s_request.scenario_types = scenario_types_;
    std::stringstream ss;
    ss << "{\"distance\": " << r << "}";
    s_request.metadata = ss.str();

    // move the ScenarioDataRequest to the EvaluationResult (which may contain
    // requests for multiple scenarios)
    result.scenario_requests.push_back(s_request);

    // log info
    RCLCPP_INFO(rclcpp::get_logger(this->getRuleName()),
                "[%s] Found relevant data: Client '%s' is %.3fm from observed area "
                "origin",
                this->getRuleName().c_str(), lidar_clients[0].c_str(), r);

    // trigger storage of data in database
    this->trigger(result);
  }
}

}  // namespace event_detector_db_recording_plugin
