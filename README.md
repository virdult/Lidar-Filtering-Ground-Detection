# Lidar Filtering & Ground Detection

A ROS 2 package designed for 64-ring 3D LiDAR perception. This node processes raw point cloud data, applies spatial cropping, and utilizes a slope-based filtering algorithm to accurately segment ground points from non-ground (obstacle) points.

Developed as a composable ROS 2 node (`rclcpp_components`) for high-performance intra-process communication.

## Features
* **Spatial Cropping:** Filters out points outside a defined bounding box (front, rear, Y-axis, and Z-axis).
* **Slope-Based Ground Segmentation:** Evaluates the height difference (`dz`) and distance (`dxy`) between adjacent LiDAR rings to determine ground continuity.
* **Voxel Grid Downsampling:** Optional downsampling for the non-ground point cloud to reduce computational load for downstream local planners.
* **Polar Coordinate Sorting:** Organizes raw Cartesian points into azimuth bins for fast, deterministic processing.

## Installation

### Prerequisites
* **OS:** Ubuntu 20.04 / 22.04
* **ROS 2:** Foxy / Humble / Iron
* **Dependencies:** PCL (Point Cloud Library), `rclcpp`, `sensor_msgs`, `pcl_conversions`

### Build Instructions
Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:virdult/Lidar-Filtering-Ground-Detection.git
```

Build the package using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_filtering --symlink-install
source install/setup.zsh
```

## Usage

Run the node using the provided launch file, which loads the configuration parameters and starts the component container:

```bash
ros2 launch lidar_filtering lidar_filtering.launch.py
```

## Configuration (`config/lidar_filtering.yaml`)

All parameters can be tuned dynamically or via the YAML file.

### I/O Topics
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `input_topic` | `/ouster/points` | The raw PointCloud2 topic from the LiDAR. |
| `ground_topic` | `/ground_points` | Output topic for the segmented ground plane. |
| `nonground_topic` | `/nonground_points` | Output topic for obstacles/non-ground points. |

### Spatial Filter Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `front_min_range` | `-30.0` | Minimum X distance in front of the vehicle. |
| `crop_min_y` / `crop_max_y`| `-7.5` / `7.5` | Lateral bounding box limits. |
| `leaf_size` | `0.05` | Voxel grid leaf size for non-ground points (set to 0 to disable). |

### Ground Segmentation Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `sensor_height` | `1.0` | Z-axis distance from the ground to the LiDAR origin. |
| `slope_threshold_deg` | `10.0` | Maximum allowable slope angle between adjacent rings to be considered ground. |
| `obstacle_height_thresh`| `0.20` | Absolute height difference that immediately classifies a point as an obstacle. |
| `global_ground_tolerance`| `0.20` | Z-axis tolerance for identifying the initial absolute ground plane. |
| `noise_floor` | `0.10` | Z-axis variance allowed for noise on flat surfaces. |

## Node Architecture
This package is built using `rclcpp_components::NodeFactory`. It can be loaded into an existing component container alongside other perception nodes (like clustering or object detection) to achieve zero-copy transport via ROS 2's intra-process communication.