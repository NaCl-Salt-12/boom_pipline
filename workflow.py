#!/usr/bin/env python3
"""
Complete ROS2 Experiment Workflow:
1. Record bag with proper naming
2. Convert to CSV
3. Push to GitHub
"""

import os
import sys
import argparse
from pathlib import Path
from experiment_manager import ExperimentManager
from bag_to_csv import BagToCSVProcessor


def run_complete_workflow(
    experiment_name,
    topics=None,
    duration=None,
    base_dir="~/ros2_experiments",
    auto_process=True,
    auto_push=False,
    repo_path=None,
):
    """
    Run the complete experiment workflow

    Args:
        experiment_name: Name of the experiment
        topics: List of topics to record (None = all)
        duration: Recording duration in seconds (None = manual stop)
        base_dir: Base directory for experiments
        auto_process: Automatically convert to CSV after recording
        auto_push: Automatically push to GitHub after processing
        repo_path: Path to git repository
    """

    # Step 1: Record bag file
    print("=" * 60)
    print("STEP 1: Recording ROS2 Bag")
    print("=" * 60)

    manager = ExperimentManager(base_dir=base_dir)
    result = manager.record_bag(experiment_name, topics=topics, duration=duration)

    if result is None:
        print("\n✗ Recording failed")
        return False

    exp_dir, filename = result
    print(f"\n✓ Recording complete: {exp_dir}")

    if not auto_process:
        print("\nTo process this data later, run:")
        print(f"  python3 bag_to_csv.py {exp_dir}")
        return True

    # Step 2: Convert to CSV
    print("\n" + "=" * 60)
    print("STEP 2: Converting to CSV")
    print("=" * 60)

    processor = BagToCSVProcessor(exp_dir)
    converted_files = processor.convert_all_topics()

    if not converted_files:
        print("\n✗ No files were converted")
        return False

    processor.create_metadata_file()
    print(f"\n✓ Conversion complete: {len(converted_files)} CSV files created")

    if not auto_push:
        print("\nTo push to GitHub later, run:")
        print(f"  python3 bag_to_csv.py {exp_dir} --push")
        return True

    # Step 3: Push to GitHub
    print("\n" + "=" * 60)
    print("STEP 3: Pushing to GitHub")
    print("=" * 60)

    success = processor.push_to_github(repo_path=repo_path)

    if success:
        print("\n✓ Complete workflow finished successfully!")
        return True
    else:
        print("\n✗ Failed to push to GitHub")
        return False


def interactive_mode():
    """Run in interactive mode with user prompts"""
    print("\n" + "=" * 60)
    print("ROS2 EXPERIMENT WORKFLOW - INTERACTIVE MODE")
    print("=" * 60)

    manager = ExperimentManager()

    # Get experiment name
    experiment_name = manager.get_experiment_info()
    if not experiment_name:
        return

    # Get topics
    print("\nTopic Selection:")
    print("1. Record all topics")
    print("2. Specify specific topics")
    choice = input("Choice (1/2) [1]: ").strip() or "1"

    topics = None
    if choice == "2":
        topics_input = input("Enter topics (space-separated): ").strip()
        topics = topics_input.split() if topics_input else None

    # Get duration
    duration_input = input(
        "\nDuration in seconds (press Enter for manual stop): "
    ).strip()
    duration = int(duration_input) if duration_input else None

    # Post-processing options
    print("\nPost-Processing Options:")
    auto_process = (
        input("Convert to CSV automatically? (y/n) [y]: ").strip().lower() != "n"
    )

    auto_push = False
    repo_path = None
    if auto_process:
        auto_push = (
            input("Push to GitHub automatically? (y/n) [n]: ").strip().lower() == "y"
        )
        if auto_push:
            repo_path = (
                input("Git repository path (press Enter for default): ").strip() or None
            )

    # Run workflow
    print("\n" + "=" * 60)
    print("STARTING WORKFLOW")
    print("=" * 60)

    run_complete_workflow(
        experiment_name=experiment_name,
        topics=topics,
        duration=duration,
        auto_process=auto_process,
        auto_push=auto_push,
        repo_path=repo_path,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Complete ROS2 Experiment Workflow",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive mode
  python3 workflow.py
  
  # Record with automatic processing
  python3 workflow.py --name my_experiment --auto-process
  
  # Record specific topics with duration
  python3 workflow.py --name sensor_test --topics /camera/image /imu/data --duration 60
  
  # Complete workflow with GitHub push
  python3 workflow.py --name final_test --auto-process --auto-push --repo /path/to/repo
        """,
    )

    parser.add_argument(
        "--name", "-n", help="Experiment name (interactive mode if not provided)"
    )

    parser.add_argument(
        "--topics", "-t", nargs="+", help="Topics to record (default: all topics)"
    )

    parser.add_argument(
        "--duration",
        "-d",
        type=int,
        help="Recording duration in seconds (default: manual stop)",
    )

    parser.add_argument(
        "--base-dir",
        "-b",
        default="~/ros2_experiments",
        help="Base directory for experiments (default: ~/ros2_experiments)",
    )

    parser.add_argument(
        "--auto-process",
        "-p",
        action="store_true",
        help="Automatically convert to CSV after recording",
    )

    parser.add_argument(
        "--auto-push",
        "-g",
        action="store_true",
        help="Automatically push to GitHub (requires --auto-process)",
    )

    parser.add_argument("--repo", "-r", help="Path to git repository (for GitHub push)")

    parser.add_argument(
        "--interactive", "-i", action="store_true", help="Force interactive mode"
    )

    args = parser.parse_args()

    # Interactive mode if no experiment name provided
    if args.interactive or args.name is None:
        interactive_mode()
    else:
        # Non-interactive mode
        success = run_complete_workflow(
            experiment_name=args.name,
            topics=args.topics,
            duration=args.duration,
            base_dir=args.base_dir,
            auto_process=args.auto_process,
            auto_push=args.auto_push,
            repo_path=args.repo,
        )

        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
