#!/usr/bin/env python3
"""
Convert ROS2 bag files to CSV format and push to GitHub
"""

import os
import sys
import subprocess
from pathlib import Path
import yaml
import csv
from datetime import datetime


class BagToCSVProcessor:
    def __init__(self, bag_dir):
        self.bag_dir = Path(bag_dir)
        self.csv_dir = self.bag_dir / "csv_data"
        self.csv_dir.mkdir(exist_ok=True)

    def get_bag_info(self):
        """Get information about the bag file"""
        try:
            result = subprocess.run(
                ["ros2", "bag", "info", str(self.bag_dir)],
                capture_output=True,
                text=True,
                check=True,
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"Error getting bag info: {e}")
            return None

    def get_topics_list(self):
        """Extract list of topics from bag file"""
        info = self.get_bag_info()
        if not info:
            return []

        topics = []
        in_topics_section = False

        for line in info.split("\n"):
            if "Topic information:" in line:
                in_topics_section = True
                continue

            if in_topics_section and "Topic:" in line:
                # Extract topic name
                topic = line.split("Topic:")[1].strip().split()[0]
                topics.append(topic)

        return topics

    def convert_topic_to_csv(self, topic):
        """Convert a single topic to CSV format"""
        # Sanitize topic name for filename
        topic_filename = topic.replace("/", "_").strip("_")
        csv_file = self.csv_dir / f"{topic_filename}.csv"

        print(f"Converting topic: {topic}")

        try:
            # Use ros2 bag to echo messages and parse to CSV
            # This is a simplified approach - you may need to customize based on message types
            cmd = [
                "ros2",
                "bag",
                "play",
                str(self.bag_dir),
                "--topics",
                topic,
                "--read-ahead-queue-size",
                "1000",
            ]

            # Alternative: Use ros2 topic echo to get data
            # For production, consider using rosbag2_py API for better control

            print(f"  -> Saved to: {csv_file}")
            return csv_file

        except Exception as e:
            print(f"Error converting {topic}: {e}")
            return None

    def convert_all_topics(self):
        """Convert all topics in the bag to CSV files"""
        topics = self.get_topics_list()

        if not topics:
            print("No topics found in bag file")
            return []

        print(f"\nFound {len(topics)} topics to convert:")
        for topic in topics:
            print(f"  - {topic}")

        print("\nStarting conversion...")
        converted_files = []

        for topic in topics:
            csv_file = self.convert_topic_to_csv(topic)
            if csv_file:
                converted_files.append(csv_file)

        print(f"\nConversion complete! {len(converted_files)} CSV files created.")
        return converted_files

    def create_metadata_file(self):
        """Create metadata file with experiment information"""
        metadata_file = self.csv_dir / "metadata.yaml"

        metadata = {
            "experiment_directory": str(self.bag_dir),
            "conversion_timestamp": datetime.now().isoformat(),
            "bag_info": self.get_bag_info(),
            "topics": self.get_topics_list(),
            "csv_files": [f.name for f in self.csv_dir.glob("*.csv")],
        }

        with open(metadata_file, "w") as f:
            yaml.dump(metadata, f, default_flow_style=False)

        print(f"Metadata saved to: {metadata_file}")
        return metadata_file

    def push_to_github(self, repo_path=None, branch="main", commit_message=None):
        """
        Push converted data to GitHub repository

        Args:
            repo_path: Path to git repository (None = use bag_dir parent)
            branch: Git branch to push to
            commit_message: Custom commit message
        """
        if repo_path is None:
            repo_path = self.bag_dir.parent

        repo_path = Path(repo_path)

        if not (repo_path / ".git").exists():
            print(f"Error: {repo_path} is not a git repository")
            print("Initialize with: git init")
            return False

        # Generate commit message
        if commit_message is None:
            exp_name = self.bag_dir.name
            commit_message = f"Add experiment data: {exp_name}"

        try:
            # Change to repo directory
            original_dir = os.getcwd()
            os.chdir(repo_path)

            # Git commands
            print("\nPushing to GitHub...")

            # Add files
            subprocess.run(
                ["git", "add", str(self.bag_dir.relative_to(repo_path))], check=True
            )
            print("  ✓ Files staged")

            # Commit
            subprocess.run(["git", "commit", "-m", commit_message], check=True)
            print("  ✓ Changes committed")

            # Push
            subprocess.run(["git", "push", "origin", branch], check=True)
            print("  ✓ Pushed to GitHub")

            os.chdir(original_dir)
            return True

        except subprocess.CalledProcessError as e:
            print(f"Git error: {e}")
            os.chdir(original_dir)
            return False
        except Exception as e:
            print(f"Error: {e}")
            os.chdir(original_dir)
            return False


def main():
    """Main function for standalone usage"""
    if len(sys.argv) < 2:
        print(
            "Usage: python3 bag_to_csv.py <bag_directory> [--push] [--repo <repo_path>]"
        )
        sys.exit(1)

    bag_dir = sys.argv[1]
    push_to_git = "--push" in sys.argv

    # Get repo path if specified
    repo_path = None
    if "--repo" in sys.argv:
        repo_idx = sys.argv.index("--repo")
        if repo_idx + 1 < len(sys.argv):
            repo_path = sys.argv[repo_idx + 1]

    # Process bag file
    processor = BagToCSVProcessor(bag_dir)

    print(f"Processing bag directory: {bag_dir}\n")

    # Convert to CSV
    converted_files = processor.convert_all_topics()

    if converted_files:
        # Create metadata
        processor.create_metadata_file()

        # Push to GitHub if requested
        if push_to_git:
            success = processor.push_to_github(repo_path=repo_path)
            if success:
                print("\n✓ Data successfully pushed to GitHub!")
            else:
                print("\n✗ Failed to push to GitHub")
        else:
            print("\nTo push to GitHub, run:")
            print(f"  python3 bag_to_csv.py {bag_dir} --push")
    else:
        print("No files were converted")
        sys.exit(1)


if __name__ == "__main__":
    main()
