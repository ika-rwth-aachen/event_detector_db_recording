# event_detector_db_recording_plugin

Action plugin for the ROS 2 [event detector framework](https://github.com/ika-rwth-aachen/event_detector) that writes buffered data to a [MongoDB](https://www.mongodb.com/) database.

See the [repository README](../README.md) for an overview, quick start, and installation instructions.

## Launch Files

### [`event_detector_db_recording_plugin.launch.py`](launch/event_detector_db_recording_plugin.launch.py)

Wraps the core `event_detector.launch.py` and loads this plugin.

| Argument | Default | Description |
| --- | --- | --- |
| `name` | `event_detector_db_recording_plugin` | node name |
| `namespace` | `""` | node namespace |
| `startup_state` | `None` | initial lifecycle state (1: unconfigured, 2: inactive, 3: active) |
| `params` | `<share>/event_detector_db_recording_plugin/config/params.yml` | path to parameter file |
| `log_level` | `info` | ROS logging level (debug, info, warn, error, fatal) |

## Analysis Rules

All rules in this plugin derive from [`RecordingRule`](include/event_detector_db_recording_plugin/RecordingRule.hpp), which connects to the database and stores the data requested by a rule. Data of [large data types](#large-data-types) is written to disk instead.

All rule parameters below are relative to the core event detector parameter [`rule_params.<RULE_NAME>.parameters`](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#parameters). Core event detector topics and parameters are [documented here](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#parameters).

- [`RecordAllRule`](#recordallrule)

### RecordAllRule

This rule records all buffered data of all enabled data types to the database. On every evaluation cycle, it stores the data that was buffered since the previous evaluation, for every client that provides an enabled data type. Each evaluation is stored as a scenario of type `recording`.

#### Subscribed Topics

None. Data to record is buffered by the event detector (configure it under `client_params.<CLIENT_NAME>.data_type_params.<DATA_TYPE>.topics`).

#### Published Topics

None.

#### Parameters

Database connection parameters are configured via the `RecordingRule` [base parameters](#base-parameters-recordingrule).

| Parameter | Type | Default | Description | Options |
| --- | --- | --- | --- | --- |
| `<DATA_TYPE>` | `bool` | `false` | whether to record a specific data type, e.g. `geometry_msgs__PoseStamped` | see third column in [`datatypes.macro`](include/event_detector_db_recording_plugin/datatypes.macro) |

### Base parameters (`RecordingRule`)

All rules of this plugin derive from `RecordingRule`, which connects to the MongoDB database on configuration and registers all connected clients in it.

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `database.name` | `string` | `db` | database name |
| `database.host` | `string` | `localhost` | database host |
| `database.port` | `int` | `27017` | database port |
| `database.user` | `string` | `""` | database user |
| `database.pass` | `string` | `""` | database user password |
| `database.large_data_root` | `string` | `""` | directory that [large data](#large-data-types) is written to; relative paths are resolved relative to the package source directory, so an absolute path is recommended |
| `database.dry_run` | `bool` | `false` | whether to perform a dry run (no data will be stored) |

## Large Data Types

Data of large data types is not stored in the database directly, but written to files below `database.large_data_root`. The database instead stores a replacement message that references the file.

| Data Type | Stored on Disk | Stored in Database |
| --- | --- | --- |
| `sensor_msgs/msg/Image` | PNG file | [`event_detector_db_recording_msgs/msg/ImageFile`](https://github.com/ika-rwth-aachen/event_detector_db_recording_msgs/blob/main/event_detector_db_recording_msgs/msg/ImageFile.msg) |
| `sensor_msgs/msg/PointCloud2` | PCD file | [`event_detector_db_recording_msgs/msg/PointCloudFile`](https://github.com/ika-rwth-aachen/event_detector_db_recording_msgs/blob/main/event_detector_db_recording_msgs/msg/PointCloudFile.msg) |

Large data types and their replacement types are defined in [`datatypes.macro`](include/event_detector_db_recording_plugin/datatypes.macro); writing to disk is implemented in [`DatabaseInterface`](include/event_detector_db_recording_plugin/DatabaseInterface.hpp).

## Custom Analysis Rules

The [`TemplateRule`](include/event_detector_db_recording_plugin/rules/template/TemplateRule.hpp) is a starting point for implementing a custom rule. It derives from `RecordingRule`, so it only needs to override `evaluate()` and call `RecordingRule::trigger(const EvaluationResult&)` to store the requested data. `TemplateRule` is not registered in [`plugins.xml`](plugins.xml) and therefore cannot be loaded as is.

On every evaluation cycle, the `TemplateRule` looks for the first client that provides `sensor_msgs/msg/PointCloud2` data and checks whether its latest `geometry_msgs/msg/PoseStamped` lies within a circular area. If so, it stores this pose and the latest point cloud of the client as a scenario of type `template`, including the distance to the area origin as metadata.

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `x` | `double` | `0.0` | x-coordinate (m) of the origin of the observed area |
| `y` | `double` | `0.0` | y-coordinate (m) of the origin of the observed area |
| `radius` | `double` | `50.0` | radius (m) of the observed area |

To support additional ROS message types, conversion functions to BSON can be generated with [`scripts/genMsgToBson.py`](scripts/README.md).
