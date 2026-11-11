// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <set>

#include "event_detector_db_recording_plugin/RecordingRule.hpp"

namespace event_detector_db_recording_plugin {

/**
 * @brief Rule to record all data buffered over each analysis period
 *
 * This rule stores all buffered data from all clients since it last ran.
 * Datatypes to store can be selected.
 */
class RecordAllRule : public RecordingRule {
 public:
  /**
   * @brief Returns the name of the rule.
   *
   * @return std::string rule name
   */
  std::string getRuleName() const override;

  /**
   * @brief Initializes the rule by setting the start of the first recording window to the current time.
   */
  void onInitialize() override;

  /**
   * @brief Loads which datatypes to record as well as the generic RecordingRule parameters.
   */
  void loadRuleParameters() override;

 protected:
  /**
   * @brief Requests all buffered data of the selected datatypes since the last evaluation for storage.
   *
   * Creates one scenario containing a request per client that provides any of the selected datatypes.
   */
  void evaluate() override;

 protected:
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION) \
  /** whether to record data of type VAR */         \
  bool VAR##_ = false;
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE

  /**
   * @brief start time of recording window, end time of previous window
   */
  rclcpp::Time start_time_;

  /**
   * @brief identified ScenarioTypes
   */
  std::set<std::string> scenario_types_ = {"recording"};
};

}  // namespace event_detector_db_recording_plugin
