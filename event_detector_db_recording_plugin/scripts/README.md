## `genMsgToBson.py`: Code Generator for ROS-to-BSON Conversions

This script can be used to automatically generate function declarations and implementations for converting ROS message objects to MongoDB BSON.
The generated source code needs to be pasted to [toBson.hpp](../include/event_detector_db_recording_plugin/toBson.hpp) and [toBson.cpp](../src/toBson.cpp), respectively.

### Usage

```
usage: genMsgToBson.py [-h] [-m {cpp,h,include,external}] namespace file

Generates C++ code for converting ROS messages to MongoDB BSON.

positional arguments:
  namespace             ROS message namespace
  file                  ROS .msg file

required arguments:
  -rv {1,2}, --ros-version {1,2}
                        ROS version (1 or 2)

optional arguments:
  -h, --help            show this help message and exit
  -m {cpp,h,include,external}, --mode {cpp,h,include,external}
                        Mode: 'cpp': function implementation; 'h': function
                        declaration; 'include': includes; 'external': external
                        namespaces
```

### Examples

Generate include-line for Pose.msg
```bash
./genMsgToBson.py -m include geometry_msgs /opt/ros/humble/share/geometry_msgs/msg/Pose.msg -rv 2
```

Generate function declaration for Pose.msg
```bash
./genMsgToBson.py -m h geometry_msgs /opt/ros/humble/share/geometry_msgs/msg/Pose.msg -rv 2
```

Generate function implementation for Pose.msg
```bash
./genMsgToBson.py -m cpp geometry_msgs /opt/ros/humble/share/geometry_msgs/msg/Pose.msg -rv 2
```

Generate list of external namespaces used in Pose.msg (helpful to know which other message packages should be processed)
```bash
./genMsgToBson.py -m external geometry_msgs /opt/ros/humble/share/geometry_msgs/msg/PoseStamped.msg -rv 2
```

Generate function implementations for all message definitions of a package (helpful to quickly paste code for multiple message types)
```bash
for m in /opt/ros/humble/share/geometry_msgs/msg/*.msg; do ./genMsgToBson.py geometry_msgs -rv 2 $m; echo ""; done
```
