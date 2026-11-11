// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <filesystem>

#include <event_detector/AnalysisRule.hpp>

#include "event_detector_db_recording_plugin/DatabaseInterface.hpp"
#include "event_detector_db_recording_plugin/common.hpp"

namespace event_detector_db_recording_plugin {

namespace ed = event_detector;

class RecordingRule : public ed::AnalysisRule {
 public:
  /**
   * @brief Loads the database configuration parameters and sets up the database connection.
   *
   * Registers all clients currently connected to the event detector in the database.
   * Derived rules should call this at the end of their own loadRuleParameters().
   */
  void loadRuleParameters() override;

 protected:
  /**
   * @brief Triggers storage of data in database.
   *
   * To be called at the end of the evaluate() function.
   *
   * @param result information on what data to store
   */
  void trigger(const EvaluationResult& result);

 private:
  /**
   * @brief Requests buffer data and stores it in database as a scenario.
   *
   * @param request information on what data to store
   */
  void requestAndStoreInDatabase(const ScenarioDataRequest& request);

  /**
   * @brief Requests specific data type samples from the buffer.
   *
   * @tparam T data type
   *
   * @param data_buffer BufferManager member holding data type
   * @param data_request data request
   * @param client_id  client id
   * @param start_time start time
   * @param end_time end time
   *
   * @return ed::string_map<std::vector<ed::Stamped<T>>> requested samples by topic
   */
  template <typename T>
  ed::string_map<std::vector<ed::Stamped<T>>> requestClientData(ed::BufferManager::DataBuffers<T>& data_buffer,
                                                                const DataRequest& data_request,
                                                                const std::string& client_id,
                                                                const rclcpp::Time& start_time,
                                                                const rclcpp::Time& end_time);

 private:
  /**
   * @brief database configuration
   */
  struct {
    std::string name;                       ///< database name
    std::string host;                       ///< database host
    int port;                               ///< database port
    std::string user;                       ///< database user
    std::string pass;                       ///< database user password
    std::filesystem::path large_data_root;  ///< root of large data folder
    bool dry_run;                           ///< whether to run in dry mode, i.e. don't touch the database
  } db_config_;

  /**
   * @brief DatabaseInterface component
   */
  std::shared_ptr<DatabaseInterface> db_;
};

}  // namespace event_detector_db_recording_plugin

#include <event_detector_db_recording_plugin/RecordingRule.tpp>