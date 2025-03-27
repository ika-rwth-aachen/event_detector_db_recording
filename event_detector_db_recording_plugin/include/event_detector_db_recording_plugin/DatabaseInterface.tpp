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

#include <rosidl_runtime_cpp/message_type_support_decl.hpp>

#include "event_detector_db_recording_plugin/toBson.hpp"

namespace event_detector_db_recording_plugin {

template <typename T>
std::pair<std::string, std::string> DatabaseInterface::getLargeDataFilePath(const T& /*msg*/,
                                                                            const std::string& scenario_id,
                                                                            const std::string& client_id,
                                                                            const std::string& topic,
                                                                            const std::string& filename) {
  std::string collection = "unknown";
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION) \
  if (typeid(T) == typeid(TYPE)) collection = #COLLECTION;
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE

  std::string safe_topic = topic;
  if (!safe_topic.empty() && safe_topic[0] == '/') safe_topic.erase(0, 1);
  std::replace(safe_topic.begin(), safe_topic.end(), '/', '-');
  std::string rel_dir = collection + "/" + scenario_id + "/" + client_id + "/" + safe_topic + "/";
  std::string abs_dir = createLargeDataDirectory(rel_dir);
  std::string rel_filepath = rel_dir + filename;
  std::string abs_filepath = abs_dir + filename;

  return {abs_filepath, rel_filepath};
}

template <typename T>
T DatabaseInterface::writeLargeDataToDisk(const T& msg, const std::string& /*scenario_id*/,
                                          const std::string& /*client_id*/, const std::string& /*topic*/) {
  return msg;
}

template <typename T>
bsoncxx::document::value DatabaseInterface::createBucketDocument(const std::vector<ed::Stamped<T>>& samples,
                                                                 const bsoncxx::oid& scenario_oid,
                                                                 const bsoncxx::oid& client_oid,
                                                                 const std::string& topic) {
  // create bucket
  // clang-format off
  return bsoncxx::builder::stream::document{}
    << "scenario_id" << scenario_oid
    << "client_id"  << client_oid
    << "timestamp" << bsoncxx::builder::stream::open_document
      << "duration" << toBson(samples.back().stamp - samples.front().stamp)
      << "first" << toBson(samples.front().stamp)
      << "last" << toBson(samples.back().stamp)
    << bsoncxx::builder::stream::close_document
    << "ros_message" << bsoncxx::builder::stream::open_document
      << "topic" << topic
      << "type" << rosidl_generator_traits::data_type<T>()
      << "name" << rosidl_generator_traits::name<T>()
    << bsoncxx::builder::stream::close_document
    << "samples"     << toBson(samples)
  << bsoncxx::builder::stream::finalize;
  // clang-format on
}

template <typename T, typename CT>
std::vector<bsoncxx::oid> DatabaseInterface::storeData(
    const ed::string_map_2d<std::vector<ed::Stamped<T>>>& client_samples, const bsoncxx::oid& scenario_oid,
    const std::string& collection) {
  std::vector<bsoncxx::document::value> docs;

  // loop over all clients for which there are samples
  for (const auto& kv : client_samples) {
    const std::string& client_id = kv.first;
    const auto& samples_by_topic = kv.second;

    // loop over all topics from this client
    for (const auto& vkv : samples_by_topic) {
      const std::string& topic = vkv.first;
      const auto& samples = vkv.second;

      RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"),
                   "DatabaseInterface received %d samples for collection %s from client "
                   "%s",
                   static_cast<int>(samples.size()), collection.c_str(), client_id.c_str());

      // collect (potentially reduced) bucket samples
      std::vector<ed::Stamped<CT>> bucket_samples;
      for (const auto& sample : samples) {
        // write data to disk, if data is considered large
        CT converted_sample;
        if (!dry_run_) converted_sample = writeLargeDataToDisk(*sample.msg, scenario_oid.to_string(), client_id, topic);

        bucket_samples.push_back({sample.stamp, std::make_shared<CT>(converted_sample)});
      }

      // create data bucket
      bsoncxx::document::value doc = createBucketDocument(bucket_samples, scenario_oid, bsoncxx::oid(client_id), topic);
      docs.push_back(doc);

      // try to find and store transforms from client base frame to data frames
      storeStaticTransformsToDataFrames(samples, client_id);
    }
  }

  // insert all samples into database
  return insert(docs, collection);
}

template <typename T, typename CT>
void DatabaseInterface::storeDataInScenario(const ed::string_map_2d<std::vector<ed::Stamped<T>>>& client_samples,
                                            const std::string& collection, const std::string& scenario_field,
                                            const bsoncxx::oid& scenario_oid, bsoncxx::document::value& scenario_doc,
                                            std::set<std::string>& client_ids, rclcpp::Time& first_stamp,
                                            rclcpp::Time& last_stamp) {
  if (!client_samples.empty()) {
    // store data in collection
    std::vector<bsoncxx::oid> oids = storeData<T, CT>(client_samples, scenario_oid, collection);

    // append data oIDs to Scenario document
    // clang-format off
    scenario_doc = bsoncxx::builder::stream::document{}
      << bsoncxx::builder::concatenate(scenario_doc.view())
      << scenario_field << toBson(oids)
    << bsoncxx::builder::stream::finalize;
    // clang-format on

    // add involved clients and adjust scenario timestamps
    for (const auto& kv : client_samples) {
      const std::string& client_id = kv.first;
      const auto& samples_by_topic = kv.second;
      for (const auto& vkv : samples_by_topic) {
        const auto& samples = vkv.second;
        client_ids.insert(client_id);
        for (const auto& sample : samples) {
          if (first_stamp.nanoseconds() == 0 || sample.stamp < first_stamp) first_stamp = sample.stamp;
          if (last_stamp.nanoseconds() == 0 || sample.stamp > last_stamp) last_stamp = sample.stamp;
        }
      }
    }
  }
}

template <typename T>
void DatabaseInterface::storeStaticTransformsToDataFrames(const std::vector<ed::Stamped<T>>& stamped_samples,
                                                          const std::string& client_id) {
  if (!stamped_samples.empty())
    storeStaticTransformsToDataFrames(stamped_samples, client_id, ed::getHasRosHeader(*stamped_samples[0].msg));
}

template <typename T>
void DatabaseInterface::storeStaticTransformsToDataFrames(const std::vector<ed::Stamped<T>>& stamped_samples,
                                                          const std::string& client_id, const ed::HasRosHeaderYes&) {
  // collect frame IDs (just in case they are not consistent across samples)
  std::vector<std::string> frame_ids;
  for (const auto& stamped_sample : stamped_samples) frame_ids.push_back(stamped_sample.msg->header.frame_id);

  // try to find and store transforms from client base frame to data frames
  for (const auto& frame_id : frame_ids) {
    // determine base frame
    std::string base_frame;
    if (tf_listener_.client_base_frames.count(client_id) > 0) {
      base_frame = tf_listener_.client_base_frames[client_id];
    } else {
      // if no base frame is registered, try to infer it from data frame
      base_frame = frame_id.substr(0, frame_id.rfind("/") + 1) + kClientBaseFrame;
    }

    // lookup and store transform from data to base frame
    if (frame_id != base_frame && kNonClientFrames.count(frame_id) == 0) {
      auto transform = requestTransform(base_frame, frame_id);
      if (transform) {
        addTransformToConnectedClient(*transform, client_id);
        if (tf_listener_.client_base_frames.count(client_id) == 0)
          // if inferred base frame worked, store it
          tf_listener_.client_base_frames[client_id] = base_frame;
      }
    }
  }
}

template <typename T>
void DatabaseInterface::storeStaticTransformsToDataFrames(const std::vector<ed::Stamped<T>>& /*stamped_samples*/,
                                                          const std::string& /*client_id*/, const ed::HasRosHeaderNo&) {
  return;
}

}  // namespace event_detector_db_recording_plugin