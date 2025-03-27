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

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "unknown"
#endif

#include <tuple>

#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mongocxx/exception/exception.hpp>
#include <mongocxx/uri.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "event_detector_db_recording_plugin/DatabaseInterface.hpp"
#include "event_detector_db_recording_plugin/toBson.hpp"

namespace event_detector_db_recording_plugin {

mongocxx::instance DatabaseInterface::instance_ = mongocxx::instance{};

DatabaseInterface::DatabaseInterface(const bool dry_run) : dry_run_(dry_run) {
  // setup transform listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_listener_.buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_listener_.listener = std::make_shared<tf2_ros::TransformListener>(*tf_listener_.buffer);
}

void DatabaseInterface::connectToDatabase(const std::string& db, const std::string& host, const int port,
                                          const std::string& user, const std::string& password) {
  if (!dry_run_) {
    // construct connection URI (with or without authentication)
    std::string uri, safe_uri;
    uri = safe_uri = "mongodb://";
    if (!user.empty()) {
      uri += user + std::string(":") + password + std::string("@");
      safe_uri += user + std::string(":") + "<PASSWORD>" + std::string("@");
    }
    std::string host_port = host + std::string(":") + std::to_string(port);
    uri += host_port;
    safe_uri += host_port;
    if (!user.empty()) {
      std::string auth_source = std::string("/?authSource=") + db;
      uri += auth_source;
      safe_uri += auth_source;
    }

    // connect to database
    try {
      db_client_ = mongocxx::client(mongocxx::uri(uri));
      db_ = db_client_[db];
      RCLCPP_INFO(rclcpp::get_logger("event_detector_db_recording_plugin"), "Database connection set up with URI '%s'",
                  safe_uri.c_str());
    } catch (mongocxx::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                  "Database connection with URI '%s' failed, will switch to dry run: %s", safe_uri.c_str(), e.what());
      dry_run_ = true;
    }
  }
}

void DatabaseInterface::setLargeDataRoot(const std::string& path) { large_data_root_ = path; }

void DatabaseInterface::createConnectedClient(const std::string& client_id, const std::string& name) {
  RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"),
               "DatabaseInterface received information about client %s", name.c_str());

  // create filter querying the ConnectedClient document
  // clang-format off
  bsoncxx::oid oid(client_id);
  bsoncxx::document::value filter = bsoncxx::builder::stream::document{}
    << "_id"  << oid
  << bsoncxx::builder::stream::finalize;

  // set upsert information
  bsoncxx::document::value doc = bsoncxx::builder::stream::document{}
    << "$setOnInsert" << bsoncxx::builder::stream::open_document
      << "name" << name
    << bsoncxx::builder::stream::close_document
  << bsoncxx::builder::stream::finalize;
  // clang-format on

  try {
    // upsert ConnectedClient document
    update(filter, doc, "ConnectedClients", true);
  } catch (mongocxx::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "ConnectedClient '%s' could not be stored in the database.", client_id.c_str());
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Inconsistency warning: The database might receive data from undefined "
                "clients.");
  }
}

void DatabaseInterface::registerClientBaseFrame(const std::string& client_id, const std::string& base_frame) {
  if (!base_frame.empty()) tf_listener_.client_base_frames[client_id] = base_frame;
}

void DatabaseInterface::addTransformToConnectedClient(const TransformMsg& transform, const std::string& client_id) {
  RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"),
               "DatabaseInterface received Transform between frames '%s' and '%s' for "
               "client %s",
               transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), client_id.c_str());

  // only store static transforms between client frames
  if (kNonClientFrames.count(transform.header.frame_id) > 0 || kNonClientFrames.count(transform.child_frame_id) > 0)
    return;

  // create filter querying the ConnectedClient document,
  // which should only match if the transform doesn't exist yet
  // clang-format off
  bsoncxx::document::value filter = bsoncxx::builder::stream::document{}
    << "_id" << bsoncxx::oid(client_id)
    << "$or" << bsoncxx::builder::stream::open_array // <=> NAND
      << bsoncxx::builder::stream::open_document
        << "transforms.header.frame_id" << bsoncxx::builder::stream::open_document
          << "$ne" << transform.header.frame_id
        << bsoncxx::builder::stream::close_document
      << bsoncxx::builder::stream::close_document
      << bsoncxx::builder::stream::open_document
        << "transforms.child_frame_id" << bsoncxx::builder::stream::open_document
          << "$ne" << transform.child_frame_id
        << bsoncxx::builder::stream::close_document
      << bsoncxx::builder::stream::close_document
    << bsoncxx::builder::stream::close_array
  << bsoncxx::builder::stream::finalize;

  // create update operation for transform info (append to transform array)
  bsoncxx::document::value doc = bsoncxx::builder::stream::document{}
    << "$push" << bsoncxx::builder::stream::open_document
      << "transforms" << toBson(transform)
    << bsoncxx::builder::stream::close_document
  << bsoncxx::builder::stream::finalize;
  // clang-format on

  try {
    // update ConnectedClient document with new transform
    update(filter, doc, "ConnectedClients");

  } catch (mongocxx::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Transform between frames '%s' and '%s' could not be stored in the "
                "database entry for client '%s', but might already be present.",
                transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), client_id.c_str());
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Inconsistency warning: The database might contain data documents with "
                "undefined frames.");
  }
}

bsoncxx::stdx::optional<TransformMsg> DatabaseInterface::requestTransform(const std::string& target_frame,
                                                                          const std::string& source_frame,
                                                                          const rclcpp::Time& time,
                                                                          const rclcpp::Duration& timeout) {
  TransformMsg transform;
  try {
    transform = tf_listener_.buffer->lookupTransform(target_frame, source_frame, time, timeout);
  } catch (tf2::TransformException& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Lookup of transform between frames '%s' and '%s' failed: %s", target_frame.c_str(),
                source_frame.c_str(), e.what());
    return bsoncxx::stdx::nullopt;
  }

  return bsoncxx::stdx::optional<TransformMsg>(transform);
}

bsoncxx::oid DatabaseInterface::createScenarioType(const std::string& name, const std::string& description) {
  // query to check if ScenarioType with same name already exists
  // clang-format off
  bsoncxx::document::value query = bsoncxx::builder::stream::document{}
    << "name" << name
  << bsoncxx::builder::stream::finalize;
  // clang-format on
  auto result = find(query, "ScenarioTypes");
  if (result) return result->view()["_id"].get_oid().value;

  // create ScenarioType document
  // clang-format off
  bsoncxx::document::value doc = bsoncxx::builder::stream::document{}
    << "name"        << name
    << "description" << description
  << bsoncxx::builder::stream::finalize;
  // clang-format on

  // insert ScenarioType document
  bsoncxx::oid oid = insert(doc, "ScenarioTypes");

  return oid;
}

void DatabaseInterface::storeScenarioData(const ScenarioData& data) {
  // create Scenario document
  bsoncxx::oid scenario_oid;
  // clang-format off
  bsoncxx::document::value doc = bsoncxx::builder::stream::document{}
    << "_id" << scenario_oid
  << bsoncxx::builder::stream::finalize;
  // clang-format on

  // add trigger information (rule name and commit hash) to scenario
  std::string commit_hash = GIT_COMMIT_HASH;
  // clang-format off
  doc = bsoncxx::builder::stream::document{}
    << bsoncxx::builder::concatenate(doc.view())
    << "trigger" << bsoncxx::builder::stream::open_document
      << "rules" << toBson(data.rules)
      << "commit" << commit_hash
    << bsoncxx::builder::stream::close_document
    << bsoncxx::builder::stream::finalize;
  // clang-format on

  // add scenario types to scenario
  std::vector<std::string> type_names;
  std::vector<bsoncxx::oid> type_oids;
  if (!data.scenario_types.empty()) {
    try {
      for (const auto& type : data.scenario_types) {
        auto type_oid = createScenarioType(type);
        type_oids.push_back(type_oid);
        type_names.push_back(type);
      }
    } catch (mongocxx::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                  "ScenarioTypes could not be stored in the database. The corresponding "
                  "scenario data will be discarded.");
      RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                  "Inconsistency warning: The database might contain ScenarioTypes "
                  "without associated Scenarios documents.");
      return;
    }
  }
  // clang-format off
  doc = bsoncxx::builder::stream::document{}
    << bsoncxx::builder::concatenate(doc.view())
    << "type_ids" << toBson(type_oids)
    << "types"    << toBson(type_names)
  << bsoncxx::builder::stream::finalize;
  // clang-format on

  // add metadata to scenario document
  nlohmann::json j_metadata;
  try {
    j_metadata = nlohmann::json::parse(data.metadata);
  } catch (nlohmann::json::parse_error& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Failed to parse scenario metadata as json, will be ignored:%s\n'%s'", e.what(), data.metadata.c_str());
    j_metadata = nlohmann::json({});
  }
  if (!j_metadata.empty()) {
    // clang-format off
    doc = bsoncxx::builder::stream::document{}
      << bsoncxx::builder::concatenate(doc.view())
      << "metadata" << toBson(j_metadata)
    << bsoncxx::builder::stream::finalize;
    // clang-format on
  }

  // collect involved client IDs and timeframe
  std::set<std::string> client_ids;
  rclcpp::Time first_stamp, last_stamp;

  try {
    std::string scenario_field;
#define DATATYPE(TYPE, LARGE_TYPE, VAR, COLLECTION)                                                           \
  /* store TYPE data */                                                                                       \
  scenario_field = #VAR;                                                                                      \
  storeDataInScenario<TYPE, LARGE_TYPE>(data.VAR, #COLLECTION, scenario_field, scenario_oid, doc, client_ids, \
                                        first_stamp, last_stamp);
#include <event_detector_db_recording_plugin/datatypes.macro>
#undef DATATYPE

  } catch (mongocxx::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Data could not be stored in the database. The corresponding Scenario "
                "document and not-yet-stored data will be discarded.");
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Inconsistency warning: The database might contain data documents "
                "referencing the non-existent Scenario document '%s'.",
                scenario_oid.to_string().c_str());
    return;
  }

  // add client IDs and timeframe to scenario
  // clang-format off
  std::vector<bsoncxx::oid> client_oids(client_ids.begin(), client_ids.end());
  doc = bsoncxx::builder::stream::document{}
    << bsoncxx::builder::concatenate(doc.view())
    << "client_ids" << toBson(client_oids)
    << "timestamp" << bsoncxx::builder::stream::open_document
      << "duration" << toBson(last_stamp - first_stamp)
      << "first" << toBson(first_stamp)
      << "last" << toBson(last_stamp)
    << bsoncxx::builder::stream::close_document
  << bsoncxx::builder::stream::finalize;
  // clang-format on

  // insert Scenario document
  try {
    insert(doc, "Scenarios");
  } catch (mongocxx::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Scenario document '%s' could not be stored in the database.", scenario_oid.to_string().c_str());
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Inconsistency warning: The database might contain data documents "
                "referencing the non-existent Scenario document '%s'.",
                scenario_oid.to_string().c_str());
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("event_detector_db_recording_plugin"), "Stored data for new scenario '%s'",
              scenario_oid.to_string().c_str());
}

std::string DatabaseInterface::createLargeDataDirectory(const std::string& subpath) {
  std::filesystem::path directory = large_data_root_;
  directory.append(subpath);
  try {
    std::filesystem::create_directories(directory);
  } catch (std::filesystem::filesystem_error& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"), "Failed to create directory '%s': %s",
                std::string(directory).c_str(), e.what());
  }

  return directory;
}

ImageFileMsg DatabaseInterface::writeLargeDataToDisk(const ImageMsg& msg, const std::string& scenario_id,
                                                     const std::string& client_id, const std::string& topic) {
  // set filename
  std::string filename = std::to_string(msg.header.stamp.sec) +
                         std::string(9 - std::to_string(msg.header.stamp.nanosec).length(), '0') +
                         std::to_string(msg.header.stamp.nanosec) + ".png";

  // get filepath
  std::string abs_filepath, rel_filepath;
  std::tie(abs_filepath, rel_filepath) = getLargeDataFilePath(msg, scenario_id, client_id, topic, filename);

  // write image to disk
  rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  try {
    cv::imwrite(abs_filepath, image);
  } catch (cv::Exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"), "Failed to write image to disk: %s", e.what());
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Inconsistency warning: The database might contain data documents "
                "referencing the non-exported data sample.");
  }
  rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "imwrite(%s): %fs", filename.c_str(),
               (t1 - t0).seconds());

  // create ImageFile message
  ImageFileMsg image_file;
  image_file.header = msg.header;
  image_file.height = msg.height;
  image_file.width = msg.width;
  image_file.file = rel_filepath;

  return image_file;
}

PointCloudFileMsg DatabaseInterface::writeLargeDataToDisk(const PointCloudMsg& msg, const std::string& scenario_id,
                                                          const std::string& client_id, const std::string& topic) {
  // set filename
  std::string filename = std::to_string(msg.header.stamp.sec) +
                         std::string(9 - std::to_string(msg.header.stamp.nanosec).length(), '0') +
                         std::to_string(msg.header.stamp.nanosec) + ".pcd";

  // get filepath
  std::string abs_filepath, rel_filepath;
  std::tie(abs_filepath, rel_filepath) = getLargeDataFilePath(msg, scenario_id, client_id, topic, filename);

  // write pointcloud to disk
  rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  pcl::PointCloud<pcl::PointXYZI> pcl;
  pcl::fromROSMsg(msg, pcl);
  try {
    pcl::io::savePCDFile(abs_filepath, pcl, true);
  } catch (pcl::IOException& e) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"), "Failed to write pointcloud to disk: %s",
                e.what());
    RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                "Inconsistency warning: The database might contain data documents "
                "referencing the non-exported data sample.");
  }
  rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "savePCDFile(%s): %fs", filename.c_str(),
               (t1 - t0).seconds());

  // create PointCloudFile message
  PointCloudFileMsg pointcloud_file;
  pointcloud_file.header = msg.header;
  pointcloud_file.num_points = msg.width * msg.height;
  pointcloud_file.file = rel_filepath;

  return pointcloud_file;
}

bsoncxx::oid DatabaseInterface::insert(const bsoncxx::document::value& doc, const std::string& collection) {
  if (dry_run_) return bsoncxx::oid();

  try {
    rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    auto result = db_[collection].insert_one(doc.view());
    rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "db[%s].insert(1): %fs", collection.c_str(),
                 (t1 - t0).seconds());
    bsoncxx::oid oid = result->inserted_id().get_oid().value;
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "Inserted %s into collection %s",
                 oid.to_string().c_str(), collection.c_str());
    return oid;

  } catch (mongocxx::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("event_detector_db_recording_plugin"),
                 "Insertion of 1 document in collection '%s' failed: %s", collection.c_str(), e.what());
    throw;
  }
}

std::vector<bsoncxx::oid> DatabaseInterface::insert(const std::vector<bsoncxx::document::value>& docs,
                                                    const std::string& collection) {
  if (dry_run_) {
    std::vector<bsoncxx::oid> oids;
    for (size_t i = 0; i < docs.size(); i++) oids.push_back(bsoncxx::oid());
    return oids;
  }

  try {
    rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    auto result = db_[collection].insert_many(docs);
    rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "db[%s].insert(%d): %fs", collection.c_str(),
                 static_cast<int>(docs.size()), (t1 - t0).seconds());
    auto inserted_ids = result->inserted_ids();
    std::vector<bsoncxx::oid> oids;
    for (const auto& id_pair : inserted_ids) {
      oids.push_back(id_pair.second.get_oid().value);
      RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "Inserted %s into collection %s",
                   oids.back().to_string().c_str(), collection.c_str());
    }
    return oids;

  } catch (mongocxx::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("event_detector_db_recording_plugin"),
                 "Insertion of %d documents in collection '%s' failed: %s", static_cast<int>(docs.size()),
                 collection.c_str(), e.what());
    throw;
  }
}

bsoncxx::stdx::optional<bsoncxx::oid> DatabaseInterface::update(const bsoncxx::document::value& filter,
                                                                const bsoncxx::document::value& doc,
                                                                const std::string& collection, bool upsert) {
  if (dry_run_) return bsoncxx::stdx::nullopt;

  try {
    mongocxx::options::update options;
    options.upsert(upsert);
    rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    auto result = db_[collection].update_one(filter.view(), doc.view(), options);
    rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "db[%s].update(1): %fs", collection.c_str(),
                 (t1 - t0).seconds());
    if (result->upserted_id()) {
      bsoncxx::oid oid = result->upserted_id()->get_oid().value;
      RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "Upserted %s into collection %s",
                   oid.to_string().c_str(), collection.c_str());
      return oid;
    } else {
      return bsoncxx::stdx::nullopt;
    }

  } catch (mongocxx::exception& e) {
    std::string op = "Update";
    if (upsert) op = "Upsert";
    RCLCPP_FATAL(rclcpp::get_logger("event_detector_db_recording_plugin"),
                 "%s of 1 document in collection '%s' failed: %s", op.c_str(), collection.c_str(), e.what());
    throw;
  }
}

bsoncxx::stdx::optional<bsoncxx::document::value> DatabaseInterface::find(const bsoncxx::document::value& filter,
                                                                          const std::string& collection) {
  if (dry_run_) return bsoncxx::stdx::nullopt;

  try {
    rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    auto result = db_[collection].find_one(filter.view());
    rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector_db_recording_plugin"), "db[%s].find(1): %fs", collection.c_str(),
                 (t1 - t0).seconds());
    return result;

  } catch (mongocxx::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("event_detector_db_recording_plugin"), "Query in collection '%s' failed: %s",
                 collection.c_str(), e.what());
    throw;
  }
}

void DatabaseInterface::storeDataInScenario(const ed::string_map_2d<std::vector<StampedPoseMsg>>& client_samples,
                                            const std::string& collection, const std::string& scenario_field,
                                            const bsoncxx::oid& scenario_oid, bsoncxx::document::value& scenario_doc,
                                            std::set<std::string>& client_ids, rclcpp::Time& first_stamp,
                                            rclcpp::Time& last_stamp) {
  // loop over all clients for which there are samples
  for (const auto& kv : client_samples) {
    const std::string& client_id = kv.first;

    // try to find and store transform from base frame to geo center frame
    if (tf_listener_.client_base_frames.count(client_id) > 0) {
      std::string base_frame = tf_listener_.client_base_frames[client_id];
      std::string geo_center_frame = base_frame.substr(0, base_frame.rfind("/") + 1) + kClientGeoCenterFrame;
      auto transform = requestTransform(base_frame, geo_center_frame);
      if (transform)
        addTransformToConnectedClient(*transform, client_id);
      else
        RCLCPP_WARN(rclcpp::get_logger("event_detector_db_recording_plugin"),
                    "Transformation to frame '%s' could not be found, but is recommended "
                    "to be stored when storing PoseStamped.",
                    geo_center_frame.c_str());
    }
  }

  // pass data on to generic function
  storeDataInScenario<PoseMsg>(client_samples, collection, scenario_field, scenario_oid, scenario_doc, client_ids,
                               first_stamp, last_stamp);
}

}  // namespace event_detector_db_recording_plugin