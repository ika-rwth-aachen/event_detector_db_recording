#!/bin/bash

set -e

pkgs=(
    builtin_interfaces
    diagnostic_msgs
    etsi_its_cam_msgs
    etsi_its_cam_ts_msgs
    etsi_its_cpm_ts_msgs
    etsi_its_denm_msgs
    etsi_its_mapem_ts_msgs
    etsi_its_spatem_ts_msgs
    etsi_its_vam_ts_msgs
    geometry_msgs
    nav_msgs
    perception_msgs
    sensor_msgs
    shape_msgs
    std_msgs
    stereo_msgs
    tf2_msgs
    trajectory_msgs
    vision_msgs
    visualization_msgs
)

upstream_pkgs=(
    event_detector_db_recording_msgs
)

rm -f ./includes.hpp ./header.hpp ./source.cpp ./external.txt
for pkg in "${pkgs[@]}"; do
    for m in /opt/ros/humble/share/${pkg}/msg/*.msg; do
        echo $m
        ../genMsgToBson.py -m include ${pkg} -rv 2 $m >> ./includes.hpp
        ../genMsgToBson.py -m h ${pkg} -rv 2 $m >> ./header.hpp
        ../genMsgToBson.py -m cpp ${pkg} -rv 2 $m >> ./source.cpp
        echo "" >> ./source.cpp
        # ./genMsgToBson.py -m external ${pkg} -rv 2 $m >> ./external.txt
    done
done
for pkg in "${upstream_pkgs[@]}"; do
    for m in /docker-ros/ws/install/event_detector_db_recording_msgs/share/${pkg}/msg/*.msg; do
        echo $m
        ../genMsgToBson.py -m include ${pkg} -rv 2 $m >> ./includes.hpp
        ../genMsgToBson.py -m h ${pkg} -rv 2 $m >> ./header.hpp
        ../genMsgToBson.py -m cpp ${pkg} -rv 2 $m >> ./source.cpp
        echo "" >> ./source.cpp
        # ./genMsgToBson.py -m external ${pkg} -rv 2 $m >> ./external.txt
    done
done
