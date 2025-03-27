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

#include <algorithm>
#include <set>

namespace event_detector_db_recording_plugin {

template <typename T>
ed::string_map<std::vector<ed::Stamped<T>>> RecordingRule::requestClientData(
    ed::BufferManager::DataBuffers<T>& data_buffer, const DataRequest& data_request, const std::string& client_id,
    const rclcpp::Time& start_time, const rclcpp::Time& end_time) {
  ed::string_map<std::vector<ed::Stamped<T>>> requested_samples_by_topic;

  // check if data type is requested
  if (data_request.request) {
    int n_samples = data_request.n_samples;
    std::set<std::string> frames = data_request.frames;
    if (frames.empty()) frames = {""};  // fetch all, if no frame specified

    // loop over requested frames
    for (const auto& frame_id : frames) {
      // fetch data from buffer
      auto samples_by_topic = data_buffer.getWithTopic(-1, start_time, end_time, client_id, frame_id);

      // loop over topics
      for (const auto& kv : samples_by_topic) {
        const std::string& topic = kv.first;
        auto& samples = kv.second;

        // cap at specified number of samples
        int n = std::min(n_samples, static_cast<int>(samples.size()));
        if (n < 0) n = samples.size();
        int offset = samples.size() - n;
        for (size_t i = offset; i < samples.size(); i++) requested_samples_by_topic[topic].push_back(samples[i]);
      }
    }
  }

  return requested_samples_by_topic;
}

}  // namespace event_detector_db_recording_plugin
