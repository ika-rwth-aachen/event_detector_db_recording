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
 * @brief Template rule to showcase how to implement a recording rule
 *
 * This template rule will check if a client that is sharing point clouds is
 * is near a specific xy-location. If such a client is found, its latest pose
 * and point cloud will be stored.
 */
class TemplateRule : public RecordingRule {
 public:
  std::string getRuleName() const override;

  void onInitialize() override;

  void loadRuleParameters() override;

 protected:
  void evaluate() override;

 protected:
  /**
   * @brief x-origin of observed area (rule parameter)
   */
  double x_ = 0.0;

  /**
   * @brief y-origin of observed area (rule parameter)
   */
  double y_ = 0.0;

  /**
   * @brief radius of observed area (rule parameter)
   */
  double radius_ = 50.0;

  /**
   * @brief identified ScenarioTypes
   */
  std::set<std::string> scenario_types_ = {"template"};
};

}  // namespace event_detector_db_recording_plugin
