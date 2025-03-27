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
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <event_detector/common.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/stdx.hpp>
#include <rclcpp/rclcpp.hpp>

#include "event_detector_db_recording_plugin/common.hpp"

namespace event_detector_db_recording_plugin {

namespace ed = event_detector;

using ImageFileMsg = event_detector_db_recording_msgs::msg::ImageFile;
using ImageMsg = sensor_msgs::msg::Image;
using PointCloudFileMsg = event_detector_db_recording_msgs::msg::PointCloudFile;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using StampedPoseMsg = ed::Stamped<geometry_msgs::msg::PoseStamped>;
using TransformMsg = geometry_msgs::msg::TransformStamped;

/**
 * @brief Component of the Event Detector responsible for database insertion
 *
 * As one of the three components of the Event Detector, the
 * DatabaseInterface handles all connections to the MongoDB database. It is
 * responsible for converting selected ROS messages to a database-compatible
 * format and storing all data as defined per the database schema.
 */
class DatabaseInterface {
 public:
  /**
   * @brief Creates a new DatabaseInterface.
   *
   * @param  dry_run  whether dry-run is enabled
   */
  DatabaseInterface(const bool dry_run = false);

  /**
   * @brief Connects to a MongoDB database.
   *
   * @param   db           database name
   * @param   host         host address
   * @param   port         port
   * @param   user         username
   * @param   password     password
   */
  void connectToDatabase(const std::string& db, const std::string& host = "localhost", const int port = 27017,
                         const std::string& user = "", const std::string& password = "");

  /**
   * @brief Sets the root directory for storing large data (eg images).
   *
   *
   * @param   path         absolute path
   *
   */
  void setLargeDataRoot(const std::string& path);

  /**
   * @brief Creates a new ConnectedClient document in the database.
   *
   * A hash of the given client name is used as the document oID. No new
   * document is inserted if the oID already exists, i.e. a client with the
   * same name already exists.
   *
   * @param   client_id   string representation of client oID
   * @param   name        client name
   */
  void createConnectedClient(const std::string& client_id, const std::string& name);

  /**
   * @brief Registers a base frame name for a specific client.
   *
   * The base frame name will be used as the source frame when looking up and
   * storing static transforms to data frames as target frames.
   *
   * @param   client_id   string representation of client oID
   * @param   base_frame  base frame name
   */
  void registerClientBaseFrame(const std::string& client_id, const std::string& base_frame);

  /**
   * @brief Inserts all data in the given ScenarioData container into the database.
   *
   * For all data included the given container, new bucket documents are
   * inserted into the corresponding database collection. The data bucket
   * documents are linked to their ConnectedClient and to a newly created
   * Scenario document.
   *
   * @param   data         data belonging to a scenario
   */
  void storeScenarioData(const ScenarioData& data);

 protected:
  /**
   * @brief Adds a Transform message to a ConnectedClient document.
   *
   * The transform is only added if its frame_id and child_frame_id differ from
   * the existing entries. Note that this way of storing transforms is only
   * targeted at static transforms. Therefore only transforms with
   * header.frame_id = 'base_link' and child_frame_id != 'map' are stored.
   *
   * @param   transform    ROS Transform message
   * @param   client_id   string representation of client oID
   */
  void addTransformToConnectedClient(const TransformMsg& transform, const std::string& client_id);

  /**
   * @brief Gets the transform between two frames using a TransformListener.
   *
   * @param   target_frame  frame to which data should be transformed
   * @param   source_frame  frame where data originated
   * @param   time          time at which transform holds (0 = latest)
   * @param   timeout       how long to block before failing
   *
   * @return  TransformMsg  transform between the frames, if found
   */
  bsoncxx::stdx::optional<TransformMsg> requestTransform(
      const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& time = rclcpp::Time(0),
      const rclcpp::Duration& timeout = rclcpp::Duration::from_seconds(0));

  /**
   * @brief Create a new ScenarioType document in the database.
   *
   * A new document is only added if the provided name does not already exist,
   * else the oID of the existing document is returned.
   *
   * @param   name          name
   * @param   description   description
   *
   * @return  bsoncxx::oid  (inserted) ScenarioType oID
   */
  bsoncxx::oid createScenarioType(const std::string& name, const std::string& description = "");

  /**
   * @brief Recursively creates directories within the root directory for large data.
   *
   * @param   subpath      relative subpath of directories to create
   *
   * @return  std::string  absolute path of deepest directory
   */
  std::string createLargeDataDirectory(const std::string& subpath);

  /**
   * @brief Generates a filepath for a large data export sample.
   *
   * @tparam  T            datatype
   *
   * @param   msg          ROS message (only type information is used)
   * @param   scenario_id  string representation of scenario oID
   * @param   client_id   string representation of client oID
   * @param   topic        ROS topic
   * @param   filename     filename with extension
   *
   * @return  std::pair<std::string, std::string>  absolute, relative filepath
   */
  template <typename T>
  std::pair<std::string, std::string> getLargeDataFilePath(const T& msg, const std::string& scenario_id,
                                                           const std::string& client_id, const std::string& topic,
                                                           const std::string& filename);

  /**
   * @brief Writes ImageMsg data to disk and creates an ImageFileMsg message.
   *
   * Large data (here the actual image) is written to disk and the original
   * message is converted to a new message type containing only metadata. The
   * provided parameters are used to store the data in a self-explaining
   * directory tree.
   *
   * @param   msg           ROS Image message
   * @param   scenario_id   string representation of scenario oID
   * @param   client_id    string representation of client oID
   * @param   topic         ROS topic
   *
   * @return  ImageFileMsg  converted message containing only metadata
   */
  ImageFileMsg writeLargeDataToDisk(const ImageMsg& msg, const std::string& scenario_id, const std::string& client_id,
                                    const std::string& topic);

  /**
   * @brief Writes PointCloudMsg data to disk and creates an PointCloudFileMsg message.
   *
   * Large data (here the actual pointcloud) is written to disk and the original
   * message is converted to a new message type containing only metadata. The
   * provided parameters are used to store the data in a self-explaining
   * directory tree.
   *
   * @param   msg           ROS PointCloud message
   * @param   scenario_id   string representation of scenario oID
   * @param   client_id    string representation of client oID
   * @param   topic         ROS topic
   *
   * @return  PointCloudFileMsg  converted message containing only metadata
   */
  PointCloudFileMsg writeLargeDataToDisk(const PointCloudMsg& msg, const std::string& scenario_id,
                                         const std::string& client_id, const std::string& topic);

  /**
   * @brief Generic dummy function intended to be called on small data messages.
   *
   * Does nothing, only returns the provided message.
   *
   * @tparam  T            ROS message type
   *
   * @param   msg          ROS message
   * @param   scenario_id  dummy
   * @param   client_id   dummy
   * @param   topic        dummy
   *
   * @return  T            untouched input msg
   */
  template <typename T>
  T writeLargeDataToDisk(const T& msg, const std::string& scenario_id, const std::string& client_id,
                         const std::string& topic);

  /**
   * @brief Inserts a BSON document into a collection of the database.
   *
   * @param   doc           BSON document
   * @param   collection    database collection
   *
   * @return  bsoncxx::oid  oID of inserted document
   */
  bsoncxx::oid insert(const bsoncxx::document::value& doc, const std::string& collection);

  /**
   * @brief Inserts BSON documents into a collection of the database.
   *
   * @param   docs         BSON documents
   * @param   collection   database collection
   *
   * @return  std::vector<bsoncxx::oid>  oIDs of inserted documents
   */
  std::vector<bsoncxx::oid> insert(const std::vector<bsoncxx::document::value>& docs, const std::string& collection);

  /**
   * @brief Updates a document in a collection of the database.
   *
   * @param   filter       BSON filter document
   * @param   doc          BSON update document
   * @param   collection   database collection
   * @param   upsert       enable upserting
   *
   * @return  bsoncxx::stdx::optional<bsoncxx::oid>  oID of upserted document
   */
  bsoncxx::stdx::optional<bsoncxx::oid> update(const bsoncxx::document::value& filter,
                                               const bsoncxx::document::value& doc, const std::string& collection,
                                               bool upsert = false);

  /**
   * @brief Finds a document in a collection of the database.
   *
   * @param   filter       BSON filter document
   * @param   collection   database collection
   *
   * @return  bsoncxx::stdx::optional<bsoncxx::document::value>  found document
   */
  bsoncxx::stdx::optional<bsoncxx::document::value> find(const bsoncxx::document::value& filter,
                                                         const std::string& collection);

  /**
   * @brief Creates a data bucket document.
   *
   * Data samples are bucketed, i.e. a single document in a data collection
   * contains all data samples from a specific client and frame belonging to a
   * particular scenario.
   *
   * @tparam  T             ROS message type
   *
   * @param   samples       ROS messages from a specific client and frame
   * @param   scenario_oid  string representation of scenario oID
   * @param   client_oid   string representation of client oID
   * @param   topic         ROS topic
   *
   * @return  bsoncxx::document::value  BSON data bucket document
   */
  template <typename T>
  bsoncxx::document::value createBucketDocument(const std::vector<ed::Stamped<T>>& samples,
                                                const bsoncxx::oid& scenario_oid, const bsoncxx::oid& client_oid,
                                                const std::string& topic);

  /**
   * @brief Inserts data of a specific type into the database.
   *
   * @tparam  T            ROS message type
   * @tparam  CT           converted ROS message type (!= T for large data)
   *
   * @param   client_samples   ROS messages by client ID by topic
   * @param   scenario_oid      string representation of scenario oID
   * @param   collection        database collection
   *
   * @return  std::vector<bsoncxx::oid>   oIDs of inserted documents
   */
  template <typename T, typename CT = T>
  std::vector<bsoncxx::oid> storeData(const ed::string_map_2d<std::vector<ed::Stamped<T>>>& client_samples,
                                      const bsoncxx::oid& scenario_oid, const std::string& collection);

  /**
   * @brief Inserts data of a specific type into the database, linked to scenario.
   *
   * @tparam  T            ROS message type
   * @tparam  CT           converted ROS message type (!= T for large data)
   *
   * @param[in]       client_samples   ROS messages by client ID by topic
   * @param[in]       collection        database collection
   * @param[in]       scenario_field    scenario field name of linked data IDs
   * @param[in]       scenario_oid      string representation of scenario oID
   * @param[in,out]   scenario_doc      scenario document containing oID
   * @param[in,out]   client_ids       involved client IDs
   * @param[in,out]   first_stamp       beginning time of scenario
   * @param[in,out]   last_stamp        end time of scenario
   */
  template <typename T, typename CT = T>
  void storeDataInScenario(const ed::string_map_2d<std::vector<ed::Stamped<T>>>& client_samples,
                           const std::string& collection, const std::string& scenario_field,
                           const bsoncxx::oid& scenario_oid, bsoncxx::document::value& scenario_doc,
                           std::set<std::string>& client_ids, rclcpp::Time& first_stamp, rclcpp::Time& last_stamp);

  /**
   * @brief Inserts PoseStamped into the database, linked to scenario.
   *
   * Under the hood, this is only a wrapper around the templated function. Since
   * the pose data is only of use if it is transformable to other frames, the
   * static transform between the client base frame and its geo center frame is
   * additionally requested.
   *
   * @param[in]       client_samples   ROS messages by client ID by topic
   * @param[in]       collection        database collection
   * @param[in]       scenario_field    scenario field name of linked data IDs
   * @param[in]       scenario_oid      string representation of scenario oID
   * @param[in,out]   scenario_doc      scenario document containing oID
   * @param[in,out]   client_ids       involved client IDs
   * @param[in,out]   first_stamp       beginning time of scenario
   * @param[in,out]   last_stamp        end time of scenario
   */
  void storeDataInScenario(const ed::string_map_2d<std::vector<StampedPoseMsg>>& client_samples,
                           const std::string& collection, const std::string& scenario_field,
                           const bsoncxx::oid& scenario_oid, bsoncxx::document::value& scenario_doc,
                           std::set<std::string>& client_ids, rclcpp::Time& first_stamp, rclcpp::Time& last_stamp);

  /**
   * @brief Looks up and stores transforms from client base frame to data frames.
   *
   * If supplied with ROS messages without header, this does nothing.
   *
   * @tparam  T  datatype
   *
   * @param   stamped_samples  stamped data samples
   * @param   client_id       client ID
   */
  template <typename T>
  void storeStaticTransformsToDataFrames(const std::vector<ed::Stamped<T>>& stamped_samples,
                                         const std::string& client_id);

  /**
   * @brief Looks up and stores transforms from client base frame to data frames.
   *
   * Actual implementation for ROS messages with header.
   *
   * @tparam  T  datatype
   *
   * @param   stamped_samples  stamped data samples
   * @param   client_id       client ID
   */
  template <typename T>
  void storeStaticTransformsToDataFrames(const std::vector<ed::Stamped<T>>& stamped_samples,
                                         const std::string& client_id, const ed::HasRosHeaderYes&);

  /**
   * @brief Looks up and stores transforms from client base frame to data frames.
   *
   * Implementation for ROS messages without header, does nothing.
   *
   * @tparam  T  datatype
   *
   * @param   stamped_samples  stamped data samples
   * @param   client_id       client ID
   */
  template <typename T>
  void storeStaticTransformsToDataFrames(const std::vector<ed::Stamped<T>>& stamped_samples,
                                         const std::string& client_id, const ed::HasRosHeaderNo&);

 protected:
  /**
   * @brief Variable necessary for using the mongocxx driver
   *
   * Must remain alive for as long as the driver is in use.
   */
  static mongocxx::instance instance_;

  /**
   * @brief Database client connection variable
   */
  mongocxx::client db_client_;

  /**
   * @brief Database variable via which all database collections can be accessed
   */
  mongocxx::database db_;

  /**
   * @brief Root directory for storing large data
   */
  std::filesystem::path large_data_root_;

  /**
   * @brief ROS transform listener
   */
  struct {
    std::unique_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    std::unordered_map<std::string, std::string> client_base_frames;
  } tf_listener_;

  /**
   * @brief whether dry-run is enabled, i.e. database will not be touched
   */
  bool dry_run_;
};

}  // namespace event_detector_db_recording_plugin

#include <event_detector_db_recording_plugin/DatabaseInterface.tpp>
