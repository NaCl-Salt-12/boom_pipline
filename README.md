# ROS2 Experiment Workflow System

A complete solution for recording, processing, and managing ROS2 bag files with automatic naming, CSV conversion, and GitHub integration.

## Features

- ✅ Automatic filename generation: `YYYYMMDD_HHMM_experiment_001`
- ✅ Experiment name validation and sanitization
- ✅ Trial number auto-increment
- ✅ Experiment history tracking
- ✅ Prevent overwriting existing data
- ✅ Convert bag files to CSV format
- ✅ Automatic GitHub push
- ✅ Both Python script and ROS2 launch file support

## Installation

1. **Install required dependencies:**

```bash
sudo apt-get install ros-humble-rosbag2 python3-yaml
pip3 install pyyaml
```

2. **Copy the scripts to your workspace:**

```bash
mkdir -p ~/ros2_ws/src/experiment_recorder
cd ~/ros2_ws/src/experiment_recorder
# Copy all Python files here
```

3. **Make scripts executable:**

```bash
chmod +x experiment_manager.py bag_to_csv.py workflow.py
```

## File Structure

```
experiment_manager.py       # Core experiment naming and bag recording
bag_to_csv.py              # Convert ROS2 bags to CSV files
workflow.py                # Complete automated workflow
experiment_launch.py       # ROS2 launch file
```

## Usage

### Method 1: Interactive Mode (Recommended)

```bash
python3 workflow.py
```

This will guide you through:

1. Selecting/entering experiment name
2. Choosing topics to record
3. Setting duration (or manual stop)
4. Auto-processing options
5. GitHub push options

### Method 2: Command Line (Automated)

**Basic recording:**

```bash
python3 workflow.py --name my_experiment
```

**Record specific topics for 60 seconds:**

```bash
python3 workflow.py --name sensor_test \
    --topics /camera/image /imu/data \
    --duration 60
```

**Complete workflow with auto-processing and GitHub push:**

```bash
python3 workflow.py --name final_test \
    --auto-process \
    --auto-push \
    --repo /path/to/your/repo
```

### Method 3: ROS2 Launch File

**Basic launch:**

```bash
ros2 launch experiment_recorder experiment_launch.py \
    experiment_name:=my_test_run
```

**With specific topics:**

```bash
ros2 launch experiment_recorder experiment_launch.py \
    experiment_name:=my_test_run \
    topics:="/camera/image /imu/data"
```

**With duration:**

```bash
ros2 launch experiment_recorder experiment_launch.py \
    experiment_name:=my_test_run \
    duration:=60
```

### Method 4: Individual Scripts

**Just record a bag:**

```bash
python3 experiment_manager.py
# Or use as a module in your own script
```

**Just convert to CSV:**

```bash
python3 bag_to_csv.py /path/to/bag_directory
```

**Convert and push to GitHub:**

```bash
python3 bag_to_csv.py /path/to/bag_directory --push --repo /path/to/repo
```

## Experiment Name Rules

- **Length:** 4-36 characters
- **Case:** Lowercase only
- **Characters:** Letters, numbers, and underscores only
- **Spaces:** Replaced with underscores automatically
- **Special characters:** Removed automatically

**Valid examples:**

- `sensor_calibration`
- `test_run_001`
- `camera_imu_sync`

**Invalid (will be auto-corrected):**

- `My Test Run` → `my_test_run`
- `Sensor-Test!` → `sensortest`
- `EX` → ❌ Too short (minimum 4 chars)

## Output Structure

```
~/ros2_experiments/
├── experiment_history.json                    # Tracks all experiments
├── 20241031_1430_sensor_test_001/
│   ├── 20241031_1430_sensor_test_001/        # ROS2 bag files
│   ├── 20241031_1430_sensor_test_001.log     # Recording log
│   └── csv_data/
│       ├── camera_image.csv
│       ├── imu_data.csv
│       └── metadata.yaml
└── 20241031_1445_sensor_test_002/
    └── ...
```

## Example Workflows

### 1. Quick Test Run

```bash
# Start recording (all topics, manual stop)
python3 workflow.py --name quick_test --auto-process

# Press Ctrl+C when done
# CSV files automatically generated
```

### 2. Automated Data Collection

```bash
# Record for 5 minutes, process, and push to GitHub
python3 workflow.py \
    --name automated_collection \
    --duration 300 \
    --auto-process \
    --auto-push \
    --repo ~/my_research_repo
```

### 3. Specific Sensors Only

```bash
# Record only camera and IMU for 2 minutes
python3 workflow.py \
    --name camera_imu_test \
    --topics /camera/image_raw /imu/data \
    --duration 120 \
    --auto-process
```

## Python API Usage

```python
from experiment_manager import ExperimentManager
from bag_to_csv import BagToCSVProcessor

# Initialize manager
manager = ExperimentManager(base_dir="~/my_experiments")

# List previous experiments
manager.list_previous_experiments()

# Record a new experiment
exp_dir, filename = manager.record_bag(
    "my_experiment",
    topics=["/camera/image", "/imu/data"],
    duration=60
)

# Process the data
processor = BagToCSVProcessor(exp_dir)
csv_files = processor.convert_all_topics()
processor.create_metadata_file()

# Push to GitHub
processor.push_to_github(repo_path="~/my_repo")
```

## GitHub Integration

### Setup

1. **Initialize git repository:**

```bash
cd ~/ros2_experiments
git init
git remote add origin https://github.com/yourusername/your-repo.git
```

2. **Configure git (if not already done):**

```bash
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

3. **Create .gitignore:**

```bash
cat > .gitignore << EOF
*.bag
*.db3
*.db3-shm
*.db3-wal
__pycache__/
EOF
```

### Automated Push

The system will automatically:

- Stage all files in the experiment directory
- Commit with message: "Add experiment data: YYYYMMDD_HHMM_experiment_001"
- Push to the specified branch (default: main)

## Troubleshooting

### "Experiment name must be between 4 and 36 characters"

- Make sure your name is at least 4 characters long
- Maximum is 36 characters

### "Cannot find git repository"

- Initialize git in your base directory: `git init`
- Or specify a different repo path with `--repo`

### "No topics found in bag file"

- Ensure ROS2 nodes are publishing during recording
- Check with: `ros2 topic list`

### Bag file not recording

- Verify ROS2 is running: `ros2 daemon status`
- Check available topics: `ros2 topic list`
- Ensure you have write permissions in the base directory

## Advanced Configuration

### Custom Base Directory

```bash
python3 workflow.py --name my_test --base-dir /data/experiments
```

### Using in Launch Files

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Your nodes here
        Node(...),

        # Start experiment recording
        ExecuteProcess(
            cmd=['python3', 'workflow.py',
                 '--name', 'automated_run',
                 '--duration', '300',
                 '--auto-process'],
            output='screen'
        )
    ])
```

## Tips

1. **Use descriptive names:** `lidar_calibration` is better than `test1`
2. **Consistent naming:** Reuse experiment names to auto-increment trials
3. **Topic selection:** Record only what you need to save disk space
4. **Duration vs Manual:** Use duration for automated runs, manual for exploration
5. **Process regularly:** Convert to CSV promptly to catch any issues early

## License

MIT License - Feel free to modify and use in your projects!
