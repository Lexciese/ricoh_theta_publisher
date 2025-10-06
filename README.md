# ricoh_theta_publisher

ROS 2 (Python) package that wraps a RICOH THETA camera stream and provides example nodes for
publishing string messages and camera frames.

## Requirements

- ROS 2 Humble or newer with `rclpy`, `std_msgs`, and `sensor_msgs` available.
- GStreamer pipeline components for the THETA stream (`thetauvcsrc`, `decodebin`, `videoconvert`).
- Python packages: `opencv-python`, `numpy`.

Install Python dependencies in your overlay if they are not provided by your ROS distribution:

```bash
pip install opencv-python numpy
```

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select ricoh_theta_publisher
source install/setup.bash
```

## Run

```bash
ros2 run ricoh_theta_publisher capture
```
