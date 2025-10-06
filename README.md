# ricoh_theta_publisher

ROS 2 (Python) package that wraps a RICOH THETA camera stream and provides example nodes for
publishing string messages and camera frames.

## Requirements

- ROS 2 Humble or newer with `rclpy`, `std_msgs`, and `sensor_msgs` available.
- GStreamer pipeline components for the THETA stream (`thetauvcsrc`, `decodebin`, `videoconvert`).
- OpenCV built with GStreamer
- Python packages: `opencv-python`, `numpy`.


## System Setup

[Linux Getting Started Guide](https://community.theta360.guide/t/linux-getting-started/10945)
(tested in Ubuntu 24.04)

```bash
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev

```

```bash
git clone https://github.com/ricohapi/libuvc-theta.git
sudo apt install libjpeg-dev
cd libuvc-theta
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..
git clone https://github.com/ricohapi/libuvc-theta-sample.git
cd libuvc-theta-sample/gst
make

# THETA must be plugged into your computer and in 
# live streaming mode

./gst_viewer
```

Build OpenCV with GStreamer ON

```bash
sudo apt install build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3-dev python3-numpy
```

```bash
git clone https://github.com/opencv/opencv.git
cd opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D WITH_GSTREAMER=ON \
      -D WITH_FFMPEG=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_DOCS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_opencv_world=OFF \
      ..
make -j$(nproc)
sudo make install
```

after installation, check if OpenCV support GStreamer

```bash
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer
```

if it still says NO, there is openCV package from apt install or pip install, remove that.


**Get the gsttehtauvc package**

```bash
git clone https://github.com/nickel110/gstthetauvc
cd gsttehtauvc/thetauvc
make
```

Set the directory path to the `GST_PLUGIN_PATH` environment variable, example:

```bash
export GST_PLUGIN_PATH=$HOME/gstthetauvc/thetauvc
```

Check installation of `gstthetauvc` with

```bash
gst-launch-1.0 thetauvcsrc ! decodebin ! videoconvert ! autovideosink
```

if it shows a window with the video, you are done


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
