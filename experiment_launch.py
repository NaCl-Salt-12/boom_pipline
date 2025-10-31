#!/usr/bin/env python3
"""
ROS2 Launch file for experiment recording with automatic naming
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import sys
import os
from pathlib import Path

# Import the experiment manager
sys.path.append(str(Path(__file__).parent))
from experiment_manager import ExperimentManager


def generate_launch_description():
    """
    Launch file for ROS2 bag recording with automatic naming

    Usage:
        ros2 launch experiment_recorder experiment_launch.py experiment_name:=my_test_run
        ros2 launch experiment_recorder experiment_launch.py experiment_name:=my_test_run topics:="/camera/image /imu/data"
    """

    # Declare launch arguments
    experiment_name_arg = DeclareLaunchArgument(
        "experiment_name",
        default_value="default_experiment",
        description="Name of the experiment (4-36 chars, lowercase, underscores for spaces)",
    )

    topics_arg = DeclareLaunchArgument(
        "topics",
        default_value="",
        description="Space-separated list of topics to record (empty = all topics)",
    )

    duration_arg = DeclareLaunchArgument(
        "duration",
        default_value="0",
        description="Recording duration in seconds (0 = until stopped manually)",
    )

    base_dir_arg = DeclareLaunchArgument(
        "base_dir",
        default_value="~/ros2_experiments",
        description="Base directory for storing experiments",
    )

    # Get launch configurations
    experiment_name = LaunchConfiguration("experiment_name")
    topics = LaunchConfiguration("topics")
    duration = LaunchConfiguration("duration")
    base_dir = LaunchConfiguration("base_dir")

    # Python script to generate filename and start recording
    record_action = ExecuteProcess(
        cmd=[
            "python3",
            "-c",
            '''
import sys
import os
from pathlib import Path
sys.path.append("'''
            + str(Path(__file__).parent)
            + """")
from experiment_manager import ExperimentManager

# Parse arguments
experiment_name = sys.argv[1]
topics_str = sys.argv[2]
duration_str = sys.argv[3]
base_dir = sys.argv[4]

# Initialize manager
manager = ExperimentManager(base_dir=base_dir)

# Sanitize and validate
sanitized = manager.sanitize_experiment_name(experiment_name)
is_valid, message = manager.validate_experiment_name(sanitized)

if not is_valid:
    print(f"ERROR: {message}", file=sys.stderr)
    sys.exit(1)

# Parse topics
topics = topics_str.split() if topics_str else None

# Parse duration
duration = int(duration_str) if duration_str != "0" else None

# Record
result = manager.record_bag(sanitized, topics=topics, duration=duration)

if result is None:
    sys.exit(1)
        """,
            experiment_name,
            topics,
            duration,
            base_dir,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            experiment_name_arg,
            topics_arg,
            duration_arg,
            base_dir_arg,
            LogInfo(msg=["Starting experiment recording: ", experiment_name]),
            record_action,
        ]
    )
