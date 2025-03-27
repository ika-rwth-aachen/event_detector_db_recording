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
