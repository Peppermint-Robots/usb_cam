# Fisheye Camera Undistortion in usb_cam

The usb_cam node now supports built-in fisheye camera undistortion. When enabled, it will publish both the raw (distorted) image and an undistorted version.

## Features

- **Automatic fisheye undistortion** using OpenCV's fisheye model
- **Supports both fisheye and standard pinhole** distortion models
- **Minimal overhead** - undistortion maps are computed once and reused
- **Publishes undistorted images** on a separate topic

## Topics Published

When undistortion is enabled, the node publishes:

| Topic | Type | Description |
|-------|------|-------------|
| `image_raw` | sensor_msgs/Image | Raw (distorted) image from camera |
| `camera_info` | sensor_msgs/CameraInfo | Calibration parameters with distortion |
| `image_rect` | sensor_msgs/Image | **Undistorted** image |
| `image_rect/camera_info` | sensor_msgs/CameraInfo | Calibration for undistorted image (no distortion) |

## Usage

### 1. Calibrate your fisheye camera

Use the provided calibration script:

```bash
cd /home/sarvesh/peppermint_ws/src/peppermint_os/ppmt_robot_driver/scripts

# Interactive collection and calibration
python3 fisheye_intrinsic_calibration.py all --output back_cam_calibration.json

# Or from a ROS 2 bag
python3 fisheye_intrinsic_calibration.py rosbag \
    --bag /path/to/your/bag \
    --topic /back_cam/color/image_raw \
    --output back_cam_calibration.json
```

This creates:
- `back_cam_calibration.json` (readable format)
- `back_cam_calibration.yaml` (ROS format) ← **You need this file**

### 2. Configure camera parameters

Edit `robot_params/config/camera_driver_params.yaml`:

```yaml
camera_config:
  activated_cameras:
    - back_cam
  
  cameras:
    back_cam:
      type: "arducam"
      video_device: "/dev/video0"
      frame_id: "back_cam_optical_frame"
      framerate: 30.0
      image_width: 640
      image_height: 480
      pixel_format: "mjpeg2rgb"
      
      # Add calibration file path
      camera_info_url: "file:///home/peppermint/calibration/back_cam_calibration.yaml"
      
      # Enable undistortion
      enable_undistortion: true
```

### 3. Launch the camera

```bash
ros2 launch ppmt_robot_driver camera_bringup.launch.py
```

### 4. Verify undistortion

Check that both topics are publishing:

```bash
# Raw (distorted) image
ros2 topic hz /your_robot_name/back_cam/color/image_raw

# Undistorted image
ros2 topic hz /your_robot_name/back_cam/color/image_rect
```

View the images:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/your_robot_name/back_cam/color/image_raw` to see the raw fisheye image, or `/your_robot_name/back_cam/color/image_rect` to see the undistorted version.

## Performance Notes

- **First frame delay**: Undistortion maps are computed on the first frame after calibration is loaded
- **Runtime overhead**: After initialization, undistortion uses fast `cv::remap()` which is ~1-2ms per frame on typical hardware
- **Memory**: Undistortion maps require approximately `2 × width × height × 4 bytes` (e.g., ~2.4 MB for 640×480)

## Distortion Models Supported

### Fisheye (Kannala-Brandt)
- **ROS distortion_model**: `equidistant` or `fisheye`
- **Coefficients**: 4 parameters (k1, k2, k3, k4)
- **Use case**: Wide-angle fisheye cameras (>170° FOV)

### Pinhole (Brown-Conrady)
- **ROS distortion_model**: `plumb_bob` or empty
- **Coefficients**: 5 parameters (k1, k2, p1, p2, k3)
- **Use case**: Standard cameras with radial/tangential distortion

The node automatically detects the model from `camera_info.distortion_model` and applies the appropriate undistortion algorithm.

## Troubleshooting

### No undistorted image published

Check logs for:
```
No valid camera calibration found. Undistortion disabled.
```

**Solution**: Ensure `camera_info_url` parameter points to a valid calibration file.

### Poor undistortion quality

- **Recalibrate**: The calibration script requires 20-30 frames with varied board positions
- **Check RMS error**: Good calibrations have RMS < 1.0 px
- **Coverage**: Ensure calibration images cover all parts of the FOV

### Unsupported encoding error

Current supported encodings:
- `rgb8`, `bgr8`
- `mono8`
- `yuyv`, `yuv422`

**Solution**: Convert your pixel format in the camera driver or modify the undistortion code to support additional encodings.

## Example: Side-by-side comparison

You can use `image_view` to view both images:

```bash
# Terminal 1: Raw image
ros2 run image_view image_view --ros-args \
  -r image:=/your_robot_name/back_cam/color/image_raw

# Terminal 2: Undistorted image
ros2 run image_view image_view --ros-args \
  -r image:=/your_robot_name/back_cam/color/image_rect
```

---

For questions or issues, refer to the [usb_cam repository](https://github.com/ros-drivers/usb_cam) or the calibration script documentation.
