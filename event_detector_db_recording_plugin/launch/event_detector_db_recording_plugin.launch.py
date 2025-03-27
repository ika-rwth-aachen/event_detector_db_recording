#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    args = [
        DeclareLaunchArgument("name", default_value="event_detector_db_recording_plugin", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("startup_state", default_value="None", description="initial lifecycle state (1: unconfigured, 2: inactive, 3: active)"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("event_detector_db_recording_plugin"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
    ]

    event_detector_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([get_package_share_directory("event_detector"), "launch", "event_detector.launch.py"])]),
        launch_arguments={
            "name": LaunchConfiguration("name"),
            "namespace": LaunchConfiguration("namespace"),
            "startup_state": LaunchConfiguration("startup_state"),
            "params": LaunchConfiguration("params"),
            "log_level": LaunchConfiguration("log_level"),
        }.items()
    )

    return LaunchDescription([
        *args,
        event_detector_launch_description
    ])
