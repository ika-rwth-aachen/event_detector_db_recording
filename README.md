# Event Detector Action Plugin | *Database Recording*

<p align="center">
  <!--<img src="https://img.shields.io/github/v/release/ika-rwth-aachen/event_detector_db_recording"/>-->
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/event_detector_db_recording"/>
  <!--<a href="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/docker-ros.yml"><img src="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/docker-ros.yml/badge.svg"/></a>-->
  <a href="https://ika-rwth-aachen.github.io/event_detector_db_recording/"><img src="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/doc.yml/badge.svg"/></a>
  <img src="https://img.shields.io/badge/ROS 2-jazzy-293754"/>
</p>

<p align="center">
⸻ <b><i><a href="#quick-start">Quick Start</a></i></b> | <b><i><a href="#analysis-rules">Analysis Rules</a></i></b> | <b><i><a href="#installation">Installation</a></i></b> | <b><i><a href="#documentation">Documentation</a></i></b> | <b><i><a href="#research-article">Research Article</a></i></b> | <b><i><a href="#acknowledgements">Acknowledgements</a></i></b> ⸻
</p>

`event_detector_db_recording` is an action plugin for the ROS 2 [event detector framework](https://github.com/ika-rwth-aachen/event_detector) that allows to write data from buffer to a database. With this action plugin, you can automatically ...
- ***buffer*** ROS messages of arbitrary message type;
- ***detect*** events by analyzing the buffer contents with existing or custom [analysis rules](#analysis-rules);
- ***trigger*** the storage of buffer contents to a MongoDB database instance.

In order to give an example, with this action plugin you can automatically ...
- buffer robot sensor data;
- detect malfunction identified by some quantity exceeding a certain threshold;
- trigger the storage of all buffered data leading up to the incident.

The example illustrates that the event detector framework is highly modular and customizable.
- You can process data of arbitrary ROS message type, including custom type definitions.
- You can use any of the existing *analysis rules* to detect an event (e.g., *value exceeding threshold*) or define your own logic to detect events by implementing a single C++ function in a custom *analysis rule*.

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> **Advanced C-ITS Use Cases** are one of many research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your advanced driver assistance and automated driving efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***


## Quick Start

> [!TIP]
> Check out the [examples repository](https://github.com/ika-rwth-aachen/event_detector_examples) to see the event detector and all of its action plugins in action!  
> The examples also give a good idea of potential use cases for the event detector.


## Analysis Rules

The concrete detection of any specific event and the concrete action that should be triggered are implemented in analysis rules of event detector action plugins. Existing analysis rules of the database recording plugin are listed here. We highlight that the action plugin is easily extensible with new developer-defined analysis rules, see [*How to implement a custom analysis rule*](#how-to-implement-a-custom-analysis-rule).

| Analysis Rule | Purpose |
| --- | --- |
| [`RecordAllRule`](./event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/rules/record_all/README.md) | record all buffered data to database |


## Installation

You can integrate the event detector with database recording plugin into your existing ROS 2 workspace by cloning the repository, installing all dependencies using [*rosdep*](http://wiki.ros.org/rosdep), [*vcstool*](https://github.com/dirk-thomas/vcstool), and a custom script for external dependencies, and then building from source.

```bash
# ROS workspace$
git clone https://github.com/ika-rwth-aachen/event_detector_db_recording.git src/event_detector_db_recording
rosdep install -r --ignore-src --from-paths src/event_detector_db_recording
vcs import src < src/event_detector_db_recording/.repos
./src/event_detector_db_recording/docker/custom.sh
colcon build --packages-up-to event_detector_db_recording --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The event detector with database recording plugin can then be launched.
```bash
# ROS workspace$
source install/setup.bash
ros2 launch event_detector_db_recording_plugin event_detector_db_recording_plugin.launch.py startup_state:=3
```

<!--
### docker-ros

The event detector with database recording plugin is also available as a Docker image, containerized through [*docker-ros*](https://github.com/ika-rwth-aachen/docker-ros).

```bash
# event_detector_db_recording$
docker run --rm \
  -v $(pwd)/event_detector_db_recording_plugin/config/params.yml:/docker-ros/ws/install/event_detector_db_recording_plugin/share/event_detector_db_recording_plugin/config/params.yml \
    ghcr.io/ika-rwth-aachen/event_detector_db_recording:latest
```
-->

## Documentation

### Code Documentation

Browsable Doxygen code documentation is available [here](https://ika-rwth-aachen.github.io/event_detector_db_recording).

### Topics, Services, Parameters

Analysis rules are separately documented, see [*Analysis Rules*](#analysis-rules).

<details><summary><i>Click to show</i></summary>

#### Subscribed Topics

Subscriptions for buffering data are specified via the core parameter [`client_params.<CLIENT_NAME>.data_type_params.<DATA_TYPE>.topics`](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#parameters). Analysis rules may define additional subscribers.

#### Published Topics

None by default. Analysis rules may define additional publishers.

#### Parameters

Parameters of the core event detector are [documented here](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#parameters). All database recording plugin rules additionally define the following parameters under `rule_params.<RULE_NAME>.parameters`. Analysis rules may define more parameters.

| Parameter | Type | Default | Description | Options |
| --- | --- | --- | --- | --- |
| `database.name` | `string` | `db` | database name |  |
| `database.host` | `string` | `localhost` | database host |  |
| `database.port` | `int` | `27017` | database port |  |
| `database.user` | `string` | `` | database user |  |
| `database.pass` | `string` | `` | database user password |  |
| `database.large_data_root` | `string` | `` | directory for large data storage (see [*Supported data types*](#supported-data-types)) |  |
| `database.dry_run` | `bool` | `false` | whether to perform a dry run (no data will be stored) |  |

</details>

### Supported data types

<details><summary><i>Click to show</i></summary>

The database recording plugin natively supports all ROS message types supported by the core event detector, see [*Supported data types*](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#supported-data-types).

The database recording plugin has the notion of *large data types*. Data of large data types is not directly stored in the database, but is written to disk instead. To give an example, `sensor_msgs/msg/Image` messages are not directly stored in the database, but images are written to PNG files on disk instead. What is written to the database, are corresponding messages of type [`event_detector_db_recording_msgs/msg/ImageFile`](https://github.com/ika-rwth-aachen/event_detector_db_recording_msgs/blob/main/event_detector_db_recording_msgs/msg/ImageFile.msg), which store the file location on disk in the database. All large data types define a corresponding replacement data type in [`datatypes.macro`](./event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/datatypes.macro) and implement the writing to disk behavior in the [`DatabaseInterface`](./event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/DatabaseInterface.hpp).

</details>

### How to implement a custom analysis rule

<details><summary><i>Click to show</i></summary>

In general, follow the [generic instructions](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#how-to-implement-a-custom-analysis-rule) for implementing a custom analysis rule in the event detector framework. The [`event_detector_db_recording_plugin::TemplateRule`](./event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/rules/template/) is a good point to get started with an analysis rule for the database recording plugin.

Note that the `TemplateRule` inherits [`event_detector_db_recording_plugin::RecordingRule`](./event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/RecordingRule.hpp) instead of directly inheriting [`event_detector::AnalysisRule`](https://github.com/ika-rwth-aachen/event_detector/blob/main/event_detector/include/event_detector/AnalysisRule.hpp). The abstract `RecordingRule` implements the entire functionality for interfacing with a MongoDB database. This way, concrete analysis rules in the database recording plugin only need to implement the event detection by overriding `event_detector::AnalysisRule::evaluate()` (e.g., `event_detector_db_recording_plugin::TemplateRule::evaluate()`) and have it call `event_detector_db_recording_plugin::RecordingRule::trigger(const EvaluationResult&)` at the end to trigger storage.

</details>


## Research Article

If you are interested in the role of event detection in modern automated driving systems and Cooperative Intelligent Transport Systems (C-ITS), please check out our associated research article on the topic and consider citing it if you are using the event detector for your own research.

> **Event Detection in C-ITS: Classification, Use Cases, and Reference Implementation**  
> *([ResearchGate](https://www.researchgate.net/publication/388993060_Event_Detection_in_C-ITS_Classification_Use_Cases_and_Reference_Implementation))*  
>
> [Lennart Reiher](https://github.com/lreiher), [Bastian Lampe](https://github.com/bastilam), [Lukas Zanger](https://github.com/lukaszanger), [Timo Woopen](https://www.ika.rwth-aachen.de/de/institut/team/fahrzeugintelligenz-automatisiertes-fahren/woopen.html), [Lutz Eckstein](https://www.ika.rwth-aachen.de/en/institute/team/univ-prof-dr-ing-lutz-eckstein.html)  
> [Institute for Automotive Engineering (ika), RWTH Aachen University](https://www.ika.rwth-aachen.de/en/)
>
> <sup>*Abstract* – The transition from traditional hardware-centric vehicles to software-defined vehicles is largely driven by a switch to modern architectural patterns of software, including service orientation and microservices. Automated driving systems (ADS), and even more so, Cooperative Intelligent Transport Systems (C-ITS), come with requirements for scalability, modularity, and adaptability that cannot be met with conventional software architectures. The complexity and dynamics of future mobility systems also suggest to employ ideas of the event-driven architecture paradigm: distributed systems need to be able to detect and respond to events in real-time and in an asynchronous manner. In this paper, we therefore stress the importance of data-driven event detection in the context of ADS and C-ITS. First, we propose a classification scheme for event-detection use cases. We then describe a diverse set of possible use cases and apply the classification scheme to a selection of concrete, innovative examples. Last, we present a modular event detection software framework that we publish as open-source software to foster further research and development of complex C-ITS use cases, but also for robotics in general.</sup>


## Acknowledgements

This work is accomplished within the projects *6GEM* (FKZ 16KISK036K), *autotech.agil* (FKZ 01IS22088A), and *UNICAR.agil* (FKZ 16EMO0284K). We acknowledge the financial support for the projects by the *Federal Ministry of Education and Research of Germany (BMBF)*.
