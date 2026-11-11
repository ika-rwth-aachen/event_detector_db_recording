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
  std::string getRuleName() const override;

  void onInitialize() override;

  void loadRuleParameters() override;

 protected:
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
