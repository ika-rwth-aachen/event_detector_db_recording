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
#include <nlohmann/json.hpp>

#include "event_detector_db_recording_plugin/RecordingRule.hpp"

namespace event_detector_db_recording_plugin {

template <typename T>
rclcpp::Time minMsgStamp2Stamp(const std::vector<ed::Stamped<T>>& samples, const ed::HasRosHeaderYes&) {
  const auto sample =
      std::min_element(samples.begin(), samples.end(), [](const ed::Stamped<T>& s1, const ed::Stamped<T>& s2) {
        return rclcpp::Time(s1.msg.header.stamp) < rclcpp::Time(s2.msg.header.stamp);
      });
  return sample->stamp;
}

template <typename T>
rclcpp::Time minMsgStamp2Stamp(const std::vector<ed::Stamped<T>>& samples, const ed::HasRosHeaderNo&) {
  return rclcpp::Time(0);
}

template <typename T>
rclcpp::Time minMsgStamp2Stamp(const std::vector<ed::Stamped<T>>& samples) {
  return minMsgStamp2Stamp(samples, ed::HasRosHeader<T>());
}

template <typename T>
rclcpp::Time maxMsgStamp2Stamp(const std::vector<ed::Stamped<T>>& samples, const ed::HasRosHeaderYes&) {
  const auto sample =
      std::max_element(samples.begin(), samples.end(), [](const ed::Stamped<T>& s1, const ed::Stamped<T>& s2) {
        return rclcpp::Time(s1.msg.header.stamp) < rclcpp::Time(s2.msg.header.stamp);
      });
  return sample->stamp;
}

template <typename T>
rclcpp::Time maxMsgStamp2Stamp(const std::vector<ed::Stamped<T>>& samples, const ed::HasRosHeaderNo&) {
  return rclcpp::Time(0);
}

template <typename T>
rclcpp::Time maxMsgStamp2Stamp(const std::vector<ed::Stamped<T>>& samples) {
  return maxMsgStamp2Stamp(samples, ed::HasRosHeader<T>());
}

void RecordingRule::loadRuleParameters() {
  this->loadRuleParameter<std::string>("database.name", db_config_.name, "db");
  this->loadRuleParameter<std::string>("database.host", db_config_.host, "localhost");
  this->loadRuleParameter<int>("database.port", db_config_.port, 27017);
  this->loadRuleParameter<std::string>("database.user", db_config_.user, "");
  this->loadRuleParameter<std::string>("database.pass", db_config_.pass, "");
  std::string large_data_root;
  this->loadRuleParameter<std::string>("database.large_data_root", large_data_root, "");
  this->loadRuleParameter<bool>("database.dry_run", db_config_.dry_run, false);

  // process filepaths
  db_config_.large_data_root = std::filesystem::path(large_data_root);
  if (!db_config_.large_data_root.has_root_path()) {
    auto tmp = std::filesystem::path(__FILE__).parent_path().parent_path();
    tmp.append(large_data_root);
    db_config_.large_data_root = tmp;
  }
  RCLCPP_DEBUG(rclcpp::get_logger(this->getRuleName()), "db_config_.large_data_root = %s",
               db_config_.large_data_root.c_str());

  // set up DatabaseInterface
  db_ = std::make_shared<DatabaseInterface>(db_config_.dry_run);

  // set up connection to database
  db_->connectToDatabase(db_config_.name, db_config_.host, db_config_.port, db_config_.user, db_config_.pass);
  db_->setLargeDataRoot(db_config_.large_data_root);

  // register connected clients in database
  const auto connected_clients = ed_->getConnectedClients();
  for (auto& client : connected_clients) {
    db_->createConnectedClient(client.id, client.name);
    db_->registerClientBaseFrame(client.id, client.base_frame);
    RCLCPP_INFO(rclcpp::get_logger(this->getRuleName()), "Registered client '%s' with ID '%s'", client.name.c_str(),
                client.id.c_str());
  }
}

void RecordingRule::trigger(const EvaluationResult& result) {
  // postprocess scenario requests
  std::vector<ScenarioDataRequest> scenario_requests;
  for (const auto& scenario_request : result.scenario_requests) {
    ScenarioDataRequest request = scenario_request;

    // add reponsible rule
    request.rules = {this->getRuleName().c_str()};

    // check if metadata is parseable
    try {
      nlohmann::json j_tmp = nlohmann::json::parse(request.metadata);
    } catch (nlohmann::json::parse_error& e) {
      RCLCPP_WARN(rclcpp::get_logger(this->getRuleName()),
                  "Failed to parse scenario metadata as json, will be "
                  "ignored:%s\n'%s'",
                  e.what(), request.metadata.c_str());
      request.metadata = "{}";
    }

    scenario_requests.push_back(request);
  }

  // request and store data
  for (const auto& request : scenario_requests) this->requestAndStoreInDatabase(request);
}

void RecordingRule::requestAndStoreInDatabase(const ScenarioDataRequest& request) {
  ScenarioData data;
  data.scenario_types = request.scenario_types;
  data.rules = request.rules;
  data.metadata = request.metadata;

  bool request_successful = true;

  // loop over all clients for which data is requested
  for (const auto& ix : request.client_requests) {
    std::string client_id = ix.first;
    const ClientDataRequest& v_request = ix.second;

    // fetch data from buffer
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION)                                                              \
                                                                                                                 \
  /* fetch requested TYPE */                                                                                     \
  auto VAR##_by_topic =                                                                                          \
      this->requestClientData(buffer_->VAR, v_request.VAR, client_id, v_request.start_time, v_request.end_time); \
  if (!VAR##_by_topic.empty()) {                                                                                 \
    for (const auto& kv : VAR##_by_topic) {                                                                      \
      const auto& samples = kv.second;                                                                           \
      auto& data_samples = data.VAR[client_id][kv.first];                                                        \
      data_samples.insert(data_samples.end(), samples.begin(), samples.end());                                   \
    }                                                                                                            \
  } else if (v_request.VAR.request) {                                                                            \
    request_successful = false;                                                                                  \
    RCLCPP_WARN(rclcpp::get_logger(this->getRuleName()),                                                         \
                "Failed to request %s from client '%s' in time range (%f, %f)", #COLLECTION, client_id.c_str(),  \
                v_request.start_time.seconds(), v_request.end_time.seconds());                                   \
  }
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE
  }

  if (!request_successful) {
    std::stringstream ss;
    for (const std::string& rule : data.rules)
      ss << "'" << rule << "'"
         << ", ";
    RCLCPP_WARN(rclcpp::get_logger(this->getRuleName()),
                "Failed to fetch requested data from buffer, will not store scenario. "
                "Data was requested by the following rules: %s",
                ss.str().c_str());
    return;
  }

  // store data in database
  if (!request.client_requests.empty()) db_->storeScenarioData(data);
}

}  // namespace event_detector_db_recording_plugin
