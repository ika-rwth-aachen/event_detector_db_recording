# Event Detector Action Plugin | *Database Recording*

<p align="center">
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <!-- <a href="https://github.com/ika-rwth-aachen/event_detector_db_recording/releases/latest"><img src="https://img.shields.io/github/v/release/ika-rwth-aachen/event_detector_db_recording"/></a> -->
  <a href="https://github.com/ika-rwth-aachen/event_detector_db_recording/blob/main/LICENSE"><img src="https://img.shields.io/github/license/ika-rwth-aachen/event_detector_db_recording"/></a>
  <br>
  <!--<a href="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/docker-ros.yml"><img src="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/docker-ros.yml/badge.svg"/></a>-->
  <a href="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/helm-oci.yml"><img src="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/helm-oci.yml/badge.svg"/></a>
  <a href="https://ika-rwth-aachen.github.io/event_detector_db_recording/"><img src="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/consistency.yml"><img src="https://github.com/ika-rwth-aachen/event_detector_db_recording/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

<p align="center">
<b><i><a href="#quick-start">Quick Start</a></i></b> | <b><i><a href="#analysis-rules">Analysis Rules</a></i></b> | <b><i><a href="#installation">Installation</a></i></b> | <b><i><a href="#documentation">Documentation</a></i></b> | <b><i><a href="#research-article">Research Article</a></i></b> | <b><i><a href="#acknowledgements">Acknowledgements</a></i></b>
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


## Analysis Rules

The concrete detection of any specific event and the concrete action that should be triggered are implemented in analysis rules of event detector action plugins. Existing analysis rules of the database recording plugin are listed here. We highlight that the action plugin is easily extensible with new developer-defined analysis rules, see [*How to implement a custom analysis rule*](#how-to-implement-a-custom-analysis-rule).

| Analysis Rule | Purpose |
| --- | --- |
| [`RecordAllRule`](./event_detector_db_recording_plugin/README.md#recordallrule) | periodically record all buffered data of selected data types to the database |


## Installation

You can integrate the event detector with database recording plugin into your existing ROS 2 workspace by cloning the repository, installing all dependencies using [*rosdep*](http://wiki.ros.org/rosdep), [*vcstool*](https://github.com/dirk-thomas/vcstool), and a custom script for external dependencies, and then building from source.

```bash
# ROS workspace$
git clone https://github.com/ika-rwth-aachen/event_detector_db_recording.git src/event_detector_db_recording
rosdep install -r --ignore-src --from-paths src/event_detector_db_recording
vcs import src < src/event_detector_db_recording/.repos
./src/event_detector_db_recording/docker/custom.sh
colcon build --packages-up-to event_detector_db_recording_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release
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

Parameters of the core event detector are [documented here](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#parameters). Each analysis rule additionally defines its own parameters under `rule_params.<RULE_NAME>.parameters`, documented in the [*plugin README*](event_detector_db_recording_plugin/README.md#analysis-rules). Parameters for the database connection (`database.*`) are shared by all rules, see [*Base parameters*](event_detector_db_recording_plugin/README.md#base-parameters-recordingrule).

</details>

### Supported data types

<details><summary><i>Click to show</i></summary>

The database recording plugin natively supports all ROS message types supported by the core event detector, see [*Supported data types*](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#supported-data-types).

Data of *large data types* (e.g., `sensor_msgs/msg/Image`, `sensor_msgs/msg/PointCloud2`) is not stored in the database directly, but written to files on disk; the database stores a replacement message referencing the file instead, see [*Large Data Types*](event_detector_db_recording_plugin/README.md#large-data-types).

</details>

### How to implement a custom analysis rule

<details><summary><i>Click to show</i></summary>

In general, follow the [generic instructions](https://github.com/ika-rwth-aachen/event_detector?tab=readme-ov-file#how-to-implement-a-custom-analysis-rule) for implementing a custom analysis rule in the event detector framework. The [`event_detector_db_recording_plugin::TemplateRule`](./event_detector_db_recording_plugin/README.md#custom-analysis-rules) is a good point to get started with an analysis rule for the database recording plugin.

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

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| 6GEM                                          | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 16KISK036K |
| [autotech.agil](https://www.autotechagil.de/) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 01IS22088A |
| [UNICAR.agil](https://www.unicaragil.de/de/)  | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 16EMO0284K |

<p>
  <img src="https://www.drought.uni-freiburg.de/stressres/images/bmftr-logo/image" height=70>
</p>