# Tutorial: Camera-LiDAR Calibration with ROS2 Bag Files

This tutorial guides you through the complete process of calibrating your camera and LiDAR using recorded ROS2 bag data.

## Table of Contents
1. [Prerequisites](#1-prerequisites)
2. [Preparing Your ROS2 Bag](#2-preparing-your-ros2-bag)
3. [Setting Up the Calibration Target](#3-setting-up-the-calibration-target)
4. [Configuration](#4-configuration)
5. [Recording Calibration Data](#5-recording-calibration-data)
6. [Running the Calibration](#6-running-the-calibration)
7. [Assessing Results](#7-assessing-results)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Prerequisites

### Required ROS2 Topics in Your Bag File

Your ROS2 bag file must contain the following topics:

- **LiDAR PointCloud**: `sensor_msgs/msg/PointCloud2`
  - Must include **ring information** (point type XYZIR)
  - If your LiDAR driver doesn't publish ring values, you'll need to modify it
  
- **Camera Image**: `sensor_msgs/msg/Image`
  - Raw or rectified camera images
  
- **Camera Information**: `sensor_msgs/msg/CameraInfo`
  - Camera intrinsic parameters and distortion coefficients

### Verify Your Bag Topics

```bash
ros2 bag info /path/to/your/bagfile
```

Check that you can see your camera and LiDAR topics with the correct message types.

### Test Topic Playback

```bash
# Play the bag
ros2 bag play /path/to/your/bagfile

# In another terminal, check topics
ros2 topic list
ros2 topic echo /your/lidar/topic --max-count 1
ros2 topic echo /your/camera/topic --max-count 1
```

---

## 2. Preparing Your ROS2 Bag

### 2.1 Recording Requirements

When recording your calibration data, ensure:

1. **Static environment** (no moving objects in the scene if possible)
2. **Good lighting conditions** for the camera
3. **Both sensors publishing** during the entire recording
4. **Synchronized timestamps** between camera and LiDAR (if possible)

### 2.2 Recording Tips

```bash
# Record specific topics only
ros2 bag record \
  /your/camera/image_raw \
  /your/camera/camera_info \
  /your/lidar/points \
  -o calibration_recording
```

### 2.3 Handling Compressed Images

If your bag uses compressed images (`sensor_msgs/msg/CompressedImage`), you'll need to republish them as raw images:

```bash
# Terminal 1: Play the bag
ros2 bag play your_bag.mcap

# Terminal 2: Republish compressed to raw
ros2 run image_transport republish compressed raw \
  --ros-args \
  --remap in/compressed:=/your/camera/image_compressed \
  --remap out:=/camera/image_raw

# Terminal 3: Run calibration (using /camera/image_raw)
```

---

## 3. Setting Up the Calibration Target

### 3.1 Chessboard Preparation

1. **Print a chessboard pattern**:
   - Recommended: A1 size (594 x 841mm) or A0 for larger spaces
   - Square size: 65-100mm depending on your setup
   - Download patterns from: https://markhedleyjones.com/projects/calibration-checkerboard-collection

2. **Mount the chessboard**:
   - Attach to a **rigid, flat, opaque backing board**
   - Ensure the chessboard is centered on the backing board
   - Keep edges parallel
   - No warping or bending

3. **Mounting configuration**:
   - Rotate the chessboard 45° (diamond orientation)
   - Use a stand with minimal protrusions
   - Ensure stability

<p align="center">
    <img width="40%" src="img/chessboardconfigexample.png">
    <br>
    <em>Example chessboard setup with labeled dimensions</em>
</p>

### 3.2 Measure Your Target

Record these measurements (you'll need them for configuration):

- **Pattern size**: Number of inner vertices (not squares)
  - Example: 8×6 means 8 vertices horizontally, 6 vertically
- **Square length**: Size of one square in millimeters
- **Board dimensions**: Width × height of the backing board in millimeters
- **Translation offset**: Distance from board center to chessboard center (x, y)

---

## 4. Configuration

### 4.1 Edit the Configuration File

Open `cfg/params.yaml` and configure:

```yaml
# 1. Camera and LiDAR Topics
camera_topic: "/your/camera/image_raw"
camera_info: "/your/camera/camera_info"
lidar_topic: "/your/lidar/points"

# 2. Chessboard Parameters
chessboard:
  pattern_size:
    width: 8      # Number of inner vertices (columns)
    height: 6     # Number of inner vertices (rows)
  square_length: 65.0  # Size of one square in mm
  
  # Backing board dimensions (mm)
  board_dimension:
    width: 910.0
    height: 650.0
  
  # Offset from board center to chessboard center (mm)
  translation_error:
    x: 0.0   # Positive = chessboard shifted right
    y: 0.0   # Positive = chessboard shifted up

# 3. (Optional) Point Cloud Filtering Bounds
# These will be adjustable in the UI, so initial values are fine
bounds:
  x_min: -5.0
  x_max: 5.0
  y_min: -5.0
  y_max: 5.0
  z_min: -2.0
  z_max: 2.0
  k: 50
  z: 1.0
  voxel_res: 0.05
```

### 4.2 Important Notes

- **Pattern size**: Count the **inner vertices**, not the squares!
  - A board with 9×7 squares has 8×6 inner vertices
- **Square length**: Measure precisely with a ruler
- **Board dimensions**: Measure the backing board, not the printed pattern
- **Translation offset**: If your chessboard isn't perfectly centered, measure the offset

---

## 5. Recording Calibration Data

### 5.1 Scene Preparation

1. Set up in a **well-lit, static environment**
2. Ensure the calibration target is visible to **both camera and LiDAR**
3. Clear the area of moving objects
4. Position yourself outside the scene when recording

### 5.2 Recording Procedure

Record multiple poses of the chessboard:

```bash
ros2 bag record \
  /your/camera/image_raw \
  /your/camera/camera_info \
  /your/lidar/points \
  -o calibration_session_$(date +%Y%m%d_%H%M%S)
```

#### Pose Guidelines

- **Quantity**: Capture 10-20 different poses
- **Distance range**: 1.5m to 5m from sensors (adjust based on your setup)
- **Spatial coverage**: Spread poses across the field of view
- **Orientation variety**: 
  - Vary yaw (rotation left/right)
  - Vary pitch (tilt up/down)
  - Include different angles (don't just move the board along one line)

#### First Pose (Important!)

Your **first capture** should be:
- Board perpendicular to the ground
- Centered in the camera view
- Clear, straight-on view
- At a medium distance (2-3m)

#### Subsequent Poses

Vary the position and orientation:
- Move closer and farther
- Shift left and right
- Tilt at different angles
- Rotate (yaw) left and right

**What to avoid:**
- All poses at the same distance
- All poses with similar board orientation
- Poses where board is too close or too far
- Unstable positions

---

## 6. Running the Calibration

### 6.1 Start Docker Container (if using Docker)

```bash
cd docker
./run.sh
docker compose exec dev bash
```

### 6.2 Launch the Calibration Tool

```bash
ros2 launch cam_lidar_calibration run_optimiser.launch.py import_samples:=false
```

This opens RViz2 with a custom **Camera-LiDAR Calibration** panel.

### 6.3 Play Your Bag File

In a **separate terminal** (outside container if using Docker):

```bash
ros2 bag play /path/to/your/calibration_bag.mcap --loop
```

Use `--loop` to replay the bag continuously while capturing samples.

**Tip**: If bag playback is slow, try:
```bash
ros2 bag play /path/to/your/bag.mcap --rate 0  # Play as fast as possible
```

### 6.4 Adjust Bounds and View Experimental Region

Before capturing the background, you need to define the region of interest where the calibration will take place. The algorithm processes only the points within these bounds, so it's crucial to set them correctly.

#### Understanding the Experimental Region

The **experimental region** is the 3D volume in your LiDAR's coordinate frame where:
- The calibration target will be placed
- Points are filtered and processed
- Background subtraction is applied

This region is visualized in RViz2 as the `experimental_region` point cloud topic.

#### Using rqt_reconfigure to Adjust Bounds

**Option 1: RViz2 Calibration Panel (Recommended)**

The RViz2 calibration panel provides convenient sliders to adjust bounds in real-time:

1. Look for the **Camera-LiDAR Calibration** panel in RViz2 (usually on the left side)
2. Use the sliders to adjust:
   - **X min/max**: Forward/backward distance from LiDAR (meters)
   - **Y min/max**: Left/right distance from LiDAR (meters)
   - **Z min/max**: Up/down distance from LiDAR (meters)
3. Watch the `experimental_region` point cloud update in real-time
4. Additional parameters:
   - **k**: Number of neighbors for outlier removal (default: 50)
   - **z**: Standard deviation threshold for outlier removal (default: 1.0)
   - **voxel_res**: Voxel resolution for background detection (default: 0.05)

**Option 2: ros2 param command line**

You can also adjust parameters from the command line:

```bash
# Adjust X bounds (forward/backward from LiDAR)
ros2 param set /feature_extraction bounds.x_min 2.0
ros2 param set /feature_extraction bounds.x_max 6.0

# Adjust Y bounds (left/right from LiDAR)
ros2 param set /feature_extraction bounds.y_min -2.0
ros2 param set /feature_extraction bounds.y_max 2.0

# Adjust Z bounds (up/down from LiDAR)
ros2 param set /feature_extraction bounds.z_min -1.5
ros2 param set /feature_extraction bounds.z_max 0.5

# Adjust outlier removal parameters
ros2 param set /feature_extraction bounds.k 50
ros2 param set /feature_extraction bounds.z 1.0

# Adjust voxel resolution for background detection
ros2 param set /feature_extraction bounds.voxel_res 0.05
```

**Option 3: rqt_reconfigure GUI**

For a more traditional parameter tuning interface:

```bash
# In a new terminal (or from host if using Docker)
ros2 run rqt_reconfigure rqt_reconfigure
```

1. Select `/feature_extraction` from the node list
2. Adjust the `bounds.*` parameters using sliders
3. Changes take effect immediately

#### Visualizing the Experimental Region in RViz2

To see the experimental region point cloud:

1. In RViz2, click **"Add"** (bottom left)
2. Select **"PointCloud2"**
3. Set the topic to `/experimental_region`
4. Adjust the display:
   - **Size (Pixels)**: 2-3 for easier viewing
   - **Color Transformer**: Intensity or RGB8
   - **Auto Size**: Enabled

The experimental region should show only the points within your defined bounds, updating in real-time as you adjust parameters.

#### Tips for Setting Bounds

1. **Start wide, then narrow down**: Begin with generous bounds and gradually tighten them
2. **Include the entire calibration area**: Make sure all potential chessboard positions fit within bounds
3. **Exclude irrelevant areas**: Remove walls, floors, ceilings, and other static objects outside the calibration zone
4. **Check all dimensions**:
   - **X**: Should cover the full depth range where you'll place the board (e.g., 2-6 meters)
   - **Y**: Should span the width of your calibration area (e.g., -3 to +3 meters)
   - **Z**: Should include the board's vertical range (e.g., -1.5 to +0.5 meters if ground is at z=0)

5. **Watch the terminal output**: You'll see messages like "Updated bounds.x_min: 2.0" confirming changes

#### Example Bounds Configuration

For a typical indoor calibration setup with a Velodyne VLP-16:

```yaml
bounds:
  x_min: 2.0    # 2 meters in front of LiDAR
  x_max: 6.0    # 6 meters in front of LiDAR
  y_min: -2.5   # 2.5 meters to the left
  y_max: 2.5    # 2.5 meters to the right
  z_min: -1.5   # 1.5 meters below LiDAR
  z_max: 0.5    # 0.5 meters above LiDAR
```

Once you're satisfied with the bounds and can see the experimental region clearly showing your calibration area, proceed to capture the background.

### 6.5 Capture Background

1. With the bounds properly set, ensure your bag is playing a frame where **no chessboard is present**
2. The experimental region should show only the static background
3. Click **"Capture Background"** in the RViz2 calibration panel

This enables automatic background subtraction to detect the chessboard.

### 6.6 Capture Samples

#### First Sample

1. Position the bag playback to show the **first pose** (board straight-on, centered)
2. Pause the bag if needed: `ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused`
3. Click **"Capture Sample"**
4. Check the terminal for **board dimension error**:
   - Good: < 30mm
   - Acceptable: 30-50mm
   - Poor: > 50mm (consider recapturing)

5. If error is too high, click **"Discard Sample"** and try again

#### Subsequent Samples

For each pose in your bag:

1. Advance/play to the next pose
2. Ensure no people or moving objects are visible
3. Click **"Capture Sample"**
4. Verify the board outline looks correct
5. Check board dimension error
6. Discard and retry if needed

**Progress**: The panel shows how many samples you've captured.

### 6.7 Optimize

Once you've captured all desired samples (10-20 recommended):

1. Click **"Optimise"**
2. Wait for optimization to complete (may take several minutes)
3. Results are saved in `data/YYYY-MM-DD_HH-MM-SS/`

**Important**: Don't skip the "Optimise" button! Samples aren't properly saved until you click it.

---

## 7. Assessing Results

### 7.1 Locate Output File

After optimization, find the output file:
```
cam_lidar_calibration/data/YYYY-MM-DD_HH-MM-SS/calibration_YYYY-MM-DD_HH-MM-SS.csv
```

### 7.2 Run Assessment

```bash
ros2 launch cam_lidar_calibration assess_results.launch.py \
  csv:=/absolute/path/to/calibration_output.csv \
  visualise:=true
```

Replace `/absolute/path/to/calibration_output.csv` with your actual file path.

### 7.3 Interpret Results

The assessment provides:

1. **Histogram plots** with Gaussian fitting for each parameter (x, y, z, roll, pitch, yaw)
2. **Final calibration parameters** printed in terminal
3. **Reprojection error** for each sample
4. **Visualization** of projected LiDAR points onto camera images (if visualise:=true)

**Good calibration indicators:**
- Low reprojection error (< 5 pixels average)
- Tight Gaussian distributions
- LiDAR points align well with image features

### 7.4 Using the Results

The final transformation is from the **camera frame** (parent) to the **LiDAR frame** (child).

Add to your robot's URDF/TF tree:
```xml
<node pkg="tf2_ros" exec="static_transform_publisher"
      args="x y z roll pitch yaw camera_frame lidar_frame" />
```

Or publish via Python/C++:
```python
import rclpy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Use your calibration results: x, y, z, roll, pitch, yaw
```

---

## 8. Troubleshooting

### 8.1 High Board Dimension Error

**Problem**: Board dimension error > 50mm

**Solutions**:
- Move the board closer or farther (optimize distance for your LiDAR)
- Ensure board is flat and not warped
- Check the board dimensions in `params.yaml` are correct
- For low-resolution LiDARs (e.g., VLP-16), ensure at least 7 rings hit the board
- Try adjusting `distance_offset_mm` in the launch file if consistently under/overestimating

### 8.2 Board Not Detected

**Problem**: No board outline appears after clicking "Capture Sample"

**Solutions**:
- Verify background was captured correctly
- Check that the board is within the x, y, z filter bounds
- Ensure sufficient LiDAR points hit the board
- Verify the board is in a different position than the background
- Check pointcloud topic is publishing: `ros2 topic echo /your/lidar/topic --max-count 1`

### 8.3 Experimental Region Not Visible

**Problem**: Can't see the `experimental_region` point cloud in RViz2

**Solutions**:
1. **Add the topic to RViz2**:
   - Click "Add" → "PointCloud2"
   - Set topic to `/experimental_region`
   - Set size to 2-3 pixels for visibility

2. **Check if points are being published**:
   ```bash
   ros2 topic hz /experimental_region
   ros2 topic echo /experimental_region --max-count 1
   ```

3. **Verify bounds are reasonable**:
   ```bash
   ros2 param get /feature_extraction bounds.x_min
   ros2 param get /feature_extraction bounds.x_max
   # Check all bounds parameters
   ```

4. **Adjust bounds to include your scene**:
   - If bounds are too narrow, no points will pass the filter
   - Widen the bounds to encompass your calibration area
   - Watch the terminal for "Updated bounds.*" messages

5. **Check frame_id matches**:
   - The experimental region uses the LiDAR's frame_id
   - Verify in RViz2: Fixed Frame should be set to your LiDAR frame

### 8.4 Parameters Not Updating in Real-Time

**Problem**: Changes to bounds parameters don't take effect immediately

**Solutions**:
1. **Check terminal output**: You should see messages like `Updated bounds.x_min: 2.0`
2. **Verify node name**: The node should be `/feature_extraction`
   ```bash
   ros2 node list | grep feature
   ```
3. **Rebuild if needed**:
   ```bash
   cd /ros2_ws
   colcon build --packages-select cam_lidar_calibration --symlink-install
   source install/setup.bash
   ```

### 8.5 ROS2 Bag Playback is Slow

**Problem**: Bag plays at < 5 Hz when it should be faster

**Solutions**:
```bash
# Option 1: Play as fast as possible
ros2 bag play your_bag.mcap --rate 0

# Option 2: Play at 2x speed
ros2 bag play your_bag.mcap --rate 2.0

# Option 3: If using compressed images, republish to raw first
ros2 run image_transport republish compressed raw \
  --ros-args --remap in/compressed:=/camera/image/compressed --remap out:=/camera/image_raw
```

### 8.6 RViz2 Doesn't Open in Docker

**Problem**: No RViz2 window appears

**Solutions**:
```bash
# On host machine, allow Docker to connect to X server
xhost +local:docker

# Then restart the container
cd docker
./run.sh
```

### 8.7 "No module named 'em'" Error

**Problem**: Build fails with Python module errors

**Solution**: This should be fixed in the latest Docker image. Rebuild:
```bash
cd docker
./build.sh -a
```

The fix uses `--system-site-packages` for the virtual environment.

### 8.8 Poor Calibration Results

**Problem**: High reprojection error or inconsistent results

**Causes and fixes**:
- **Insufficient pose variety**: Capture more diverse poses
- **All poses at similar distance**: Spread the distance range (1.5-5m)
- **Board too small/far**: LiDAR can't see enough detail
- **Warped chessboard**: Print must be flat
- **Moving board during capture**: Ensure stability
- **Poor camera calibration**: Run camera calibration first using `camera_calibration` package

### 8.9 Getting Better Results

**Tips for optimal calibration**:
1. Use a larger chessboard (A1 or A0 size)
2. Capture 15-20 samples with good variety
3. Ensure at least 7 LiDAR rings hit the board (for Velodyne VLP-16)
4. Calibrate camera intrinsics first using:
   ```bash
   ros2 run camera_calibration cameracalibrator \
     --size 7x9 --square 0.02 --no-service-check \
     --ros-args -r image:=/camera/image_raw
   ```
5. Use consistent lighting
6. Verify LiDAR ring information is present

---

## Quick Reference Commands

```bash
# Check bag contents
ros2 bag info your_bag.mcap

# Play bag
ros2 bag play your_bag.mcap --loop

# Start calibration
ros2 launch cam_lidar_calibration run_optimiser.launch.py import_samples:=false

# Adjust bounds (in real-time)
ros2 param set /feature_extraction bounds.x_min 2.0
ros2 param set /feature_extraction bounds.x_max 6.0
ros2 param set /feature_extraction bounds.y_min -2.5
ros2 param set /feature_extraction bounds.y_max 2.5
ros2 param set /feature_extraction bounds.z_min -1.5
ros2 param set /feature_extraction bounds.z_max 0.5

# Or use rqt_reconfigure GUI
ros2 run rqt_reconfigure rqt_reconfigure

# Assess results
ros2 launch cam_lidar_calibration assess_results.launch.py \
  csv:=/path/to/output.csv \
  visualise:=true

# Republish compressed images
ros2 run image_transport republish compressed raw \
  --ros-args \
  --remap in/compressed:=/camera/image/compressed \
  --remap out:=/camera/image_raw
```

---

## Additional Resources

- [ROS2 Bag Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [Camera Calibration Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Camera-Calibration.html)
- [Original Paper](https://arxiv.org/abs/2103.12287)
- [Docker README](docker/README.md) for detailed container usage

---

For more information, see the main [README.md](README.md).

If you encounter issues not covered here, please [open an issue](https://github.com/acfr/cam_lidar_calibration/issues) on GitHub.
