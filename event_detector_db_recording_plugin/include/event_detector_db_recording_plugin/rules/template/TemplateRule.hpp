// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

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
