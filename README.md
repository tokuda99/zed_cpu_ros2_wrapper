# zed_cpu_ros2_wrapper

## Prerequisites
- ### Stereo camera: ZED 2, ZED, ZED Mini
- ### Linux OS
- ### GCC (v7.5+)
- ### CMake (v3.1+)
- ### OpenCV (v3.4.0+)
- ### ROS2 (tested on galactic/humble)

## Requirements package
- ### build-essential (for GCC compiler and build tools)
- ### cmake
- ### libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev
- ### libboost-dev
- ### libopencv-dev

## Usage

1. ### Clone this repo in ros workspace

2. ### Build and source setup.bash

3. ### Run launch file
```bash
ros2 launch zed_cpu_ros2_wrapper zed_cpu_ros2_launch.py resolution:=<RESOLUTION> fps:=<FPS> publish_rate:=<PUBLISH_RATE>
```
***Note*** : FPS needs to be greater than PUBLISH_RATE
```bash
ros2 launch zed_cpu_ros2_wrapper zed_cpu_ros2_launch.py resolution:=HD1080 fps:=15 publish_rate:=10.0
```

|Parameter                     |Description                                                |Type    |Default                    |Option                          |
|------------------------------|-----------------------------------------------------------|--------|---------------------------|--------------------------------|
|`resolution`                  |Image resolution                                           |`string`|`HD1080`                   |`HD2K`, `HD1080`, `HD720`, `VGA`|
|`fps`                         |Camera buffer update frame rate                            |`int`   |`15`                       |`100`, `60`, `30`, `15`         |
|`publish_rate`                |Image msg publish frame rate                               |`double`|`10`                       |                                |
|`use_camera_buffer_timestamps`|Change timestamp source from ROS TIME to camera buffer time|`bool`  |`False`                    |                                |
|`left_image_topic_name`       |Image topic name of left camera                            |`string`|`zed/left_camera/image_raw`|                                |
|`right_image_topic_name`      |Image topic name of right camera                           |`string`|`zed/left_camera/image_raw`|                                |
|`left_camera_frame_id`        |Frame id of left camera                                    |`string`|`left_camera`              |                                |
|`right_camera_frame_id`       |Frame id of right camera                                   |`string`|`left_camera`              |                                |
|`use_sensor_data_qos`         |Use sensor data qos                                        |`bool`  |`False`                    |                                |