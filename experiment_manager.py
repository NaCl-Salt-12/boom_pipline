#!/usr/bin/env python3
"""
ROS2 Experiment Manager - Handles experiment naming, bag recording, and data processing
"""

import os
import re
import json
import subprocess
from datetime import datetime
from pathlib import Path


class ExperimentManager:
    def __init__(
        self, base_dir="~/ros2_experiments", history_file="experiment_history.json"
    ):
        self.base_dir = Path(base_dir).expanduser()
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.history_file = self.base_dir / history_file
        self.history = self._load_history()

    def _load_history(self):
        """Load experiment history from JSON file"""
        if self.history_file.exists():
            with open(self.history_file, "r") as f:
                return json.load(f)
        return {}

    def _save_history(self):
        """Save experiment history to JSON file"""
        with open(self.history_file, "w") as f:
            json.dump(self.history, f, indent=2)

    def validate_experiment_name(self, name):
        """
        Validate experiment name:
        - 4 to 36 characters
        - Lowercase
        - Underscores allowed, no spaces
        - No special characters (only alphanumeric and underscore)
        """
        if not name:
            return False, "Experiment name cannot be empty"

        if len(name) < 4 or len(name) > 36:
            return False, "Experiment name must be between 4 and 36 characters"

        if name != name.lower():
            return False, "Experiment name must be lowercase"

        if not re.match(r"^[a-z0-9_]+$", name):
            return (
                False,
                "Experiment name can only contain lowercase letters, numbers, and underscores",
            )

        if " " in name:
            return False, "Experiment name cannot contain spaces (use underscores)"

        return True, "Valid"

    def sanitize_experiment_name(self, name):
        """Convert user input to valid experiment name"""
        # Convert to lowercase
        name = name.lower()
        # Replace spaces with underscores
        name = name.replace(" ", "_")
        # Remove special characters, keep only alphanumeric and underscore
        name = re.sub(r"[^a-z0-9_]", "", name)
        return name

    def get_next_trial_number(self, experiment_name):
        """Get the next trial number for an experiment"""
        if experiment_name in self.history:
            return self.history[experiment_name] + 1
        return 1

    def generate_filename(self, experiment_name, trial_number=None):
        """
        Generate filename following format: YYYYMMDD_HHMM_experiment_001
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")

        if trial_number is None:
            trial_number = self.get_next_trial_number(experiment_name)

        filename = f"{timestamp}_{experiment_name}_{trial_number:03d}"
        return filename, trial_number

    def create_experiment_directory(self, filename):
        """Create directory for experiment data"""
        exp_dir = self.base_dir / filename
        exp_dir.mkdir(parents=True, exist_ok=True)
        return exp_dir

    def record_bag(self, experiment_name, topics=None, duration=None):
        """
        Record ROS2 bag file with proper naming

        Args:
            experiment_name: Name of the experiment
            topics: List of topics to record (None = all topics)
            duration: Duration in seconds (None = manual stop)
        """
        # Validate and sanitize name
        sanitized_name = self.sanitize_experiment_name(experiment_name)
        is_valid, message = self.validate_experiment_name(sanitized_name)

        if not is_valid:
            print(f"Error: {message}")
            return None

        # Generate filename and trial number
        filename, trial_number = self.generate_filename(sanitized_name)

        # Create experiment directory
        exp_dir = self.create_experiment_directory(filename)
        bag_path = exp_dir / filename

        # Update history
        self.history[sanitized_name] = trial_number
        self._save_history()

        # Build ros2 bag record command
        cmd = ["ros2", "bag", "record", "-o", str(bag_path)]

        if topics:
            cmd.extend(topics)
        else:
            cmd.append("-a")  # Record all topics

        if duration:
            cmd.extend(["--max-duration", str(duration)])

        print(f"Starting recording: {filename}")
        print(f"Directory: {exp_dir}")
        print(f"Command: {' '.join(cmd)}")

        # Create log file
        log_file = exp_dir / f"{filename}.log"
        with open(log_file, "w") as f:
            f.write(f"Experiment: {sanitized_name}\n")
            f.write(f"Trial: {trial_number}\n")
            f.write(f"Timestamp: {datetime.now().isoformat()}\n")
            f.write(f"Command: {' '.join(cmd)}\n")

        try:
            subprocess.run(cmd, cwd=exp_dir)
            return exp_dir, filename
        except KeyboardInterrupt:
            print("\nRecording stopped by user")
            return exp_dir, filename
        except Exception as e:
            print(f"Error during recording: {e}")
            return None

    def list_previous_experiments(self):
        """List all previous experiment names with trial counts"""
        if not self.history:
            print("No previous experiments found")
            return []

        print("\nPrevious experiments:")
        for exp_name, trial_count in sorted(self.history.items()):
            print(f"  - {exp_name}: {trial_count} trial(s)")

        return list(self.history.keys())

    def get_experiment_info(self):
        """Interactive function to get experiment information from user"""
        print("\n=== ROS2 Experiment Manager ===\n")

        # Show previous experiments
        previous = self.list_previous_experiments()

        # Get experiment name
        if previous:
            print("\nEnter experiment name (or select from above):")
        else:
            print("\nEnter new experiment name:")

        print(
            "Requirements: 4-36 chars, lowercase, use _ for spaces, alphanumeric only"
        )
        experiment_name = input("Experiment name: ").strip()

        # Sanitize and validate
        sanitized = self.sanitize_experiment_name(experiment_name)
        is_valid, message = self.validate_experiment_name(sanitized)

        if not is_valid:
            print(f"\nError: {message}")
            if sanitized and sanitized != experiment_name:
                print(f"Sanitized version would be: '{sanitized}'")
            return None

        if sanitized != experiment_name:
            print(f"Using sanitized name: '{sanitized}'")

        # Show next trial number
        next_trial = self.get_next_trial_number(sanitized)
        filename, _ = self.generate_filename(sanitized)

        print(f"\nNext trial number: {next_trial}")
        print(f"Generated filename: {filename}")
        print(f"Bag will be saved to: {self.base_dir / filename}/")

        return sanitized


def main():
    """Main function for standalone usage"""
    manager = ExperimentManager()

    # Get experiment info interactively
    experiment_name = manager.get_experiment_info()

    if experiment_name:
        print("\nReady to record. Would you like to:")
        print("1. Record all topics")
        print("2. Specify topics")
        choice = input("Choice (1/2): ").strip()

        topics = None
        if choice == "2":
            topics_input = input("Enter topics (space-separated): ").strip()
            topics = topics_input.split()

        duration_input = input(
            "Duration in seconds (press Enter for manual stop): "
        ).strip()
        duration = int(duration_input) if duration_input else None

        # Start recording
        result = manager.record_bag(experiment_name, topics=topics, duration=duration)

        if result:
            exp_dir, filename = result
            print(f"\nRecording complete!")
            print(f"Data saved to: {exp_dir}")
            print(f"\nReady for processing with bag_to_csv.py")


if __name__ == "__main__":
    main()
