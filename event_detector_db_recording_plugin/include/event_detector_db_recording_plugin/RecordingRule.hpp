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

#include <filesystem>

#include <event_detector/AnalysisRule.hpp>

#include "event_detector_db_recording_plugin/DatabaseInterface.hpp"
#include "event_detector_db_recording_plugin/common.hpp"

namespace event_detector_db_recording_plugin {

namespace ed = event_detector;

class RecordingRule : public ed::AnalysisRule {
 public:
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