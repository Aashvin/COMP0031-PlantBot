# YOLO installation guide

YOLO packages used in this project needs to be compiled and sourced correctly
in order for the plantbot to use it. This installations covers additionally steps or materials that needs to be done.
For detailed steps, please follow the installation guide from the package repository.

## Environments

We have tested the installation on the following environments
1. Ubuntu 20.04 and ROS Noetic with CUDA available


## YOLOv2 and YOLOv3

The repo [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros) has already provide support for YOLOv2, YOLOv2-tiny and YOLOv3.
The corresponding weights should be downloaded automatically during `catkin_make` in the directory `darknet_ros/yolo_network_config/weights/`.
**If not,** use the following command to download.

```shell
cd {YOUR_WORKSPACE}/src/darknet_ros/darknet_ros/yolo_network_config/weights/
wget http://pjreddie.com/media/files/yolov2.weights
wget http://pjreddie.com/media/files/yolov2-tiny.weights
wget http://pjreddie.com/media/files/yolov3.weights
```

YOLOv3-tiny is not included by default. To enable it, use the following steps:
1. copy the [yolo-cofigs/yolov3-tiny.cfg](./yolov3-tiny.cfg) to `darknet_ros/yolo_network_config/cfg/` by the command:
    ```shell
    cp yolo-configs/yolov3-tiny.cfg {YOURWORKSPACE}/src/darknet_ros/darknet_ros/yolo_network_config/cfg/
    ```
2. copy the [yolo-configs/yolov3-tiny.yaml](./yolov3-tiny.yaml) to `darknet_ros/config/` by the command:
    ``` shell
    cp yolo-configs/yolov3-tiny.cfg {YOURWORKSPACE}/src/darknet_ros/darknet_ros/config/
    ```
3. copy the [yolo-configs/yolov3-tiny.launch](./yolov3-tiny.launch) to `darknet_ros/launch` by the command:
    ```shell
    cp yolo-configs/yolov3-tiny.launch {YOURWORKSPACE}/src/darknet_ros/darknet_ros/launch
    ```
4. Download yolov3-tiny weight by commands:
    ```shell
    cd {YOURWORKSPACE}/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    wget http://pjreddie.com/media/files/yolov3-tiny.weights
    ```

### Run nodes
First, compile and source:
```
catkin_make
source devel/setup.bash # (or setup.zsh)
```

To run different versions of yolo, you can either change the [plantbot/launch/darknet_ros.launch](../plantbot/launch/darknet_ros.launch) line 14 to the correct launch file or you can start yolo node separatedly by one of the following command:
```
roslaunch darknet_ros darknet_ros.launch # this starts yolov2-tiny
roslaunch darknet_ros yolo_v3.launch
roslaunch darknet_ros yolov3-tiny.launch
```


## YOLOv4
Since the original repo does not provide the support for yolov4, we have created a new repo [t1mkhunan9/yolov4-ros-noetic](https://github.com/t1mkhuan9/yolov4-ros-noetic). This repo has updated the version of [darknet](https://github.com/AlexeyAB/darknet/tree/8a0bf84c19e38214219dbd3345f04ce778426c57) to the latest one and has modified the code in `darknet_ros` to provide support for yolov4 on ros. Thus the compatiblity of this repo to the original repo is not tested. It is recommanded to **only** run yolo_v4 using this repo.

Detailed instructions has been provided in the repo.

### Run nodes
First, compile and source:
```
catkin_make
source devel/setup.bash # (or setup.zsh)
```
To run different versions of yolo, you can either change the [plantbot/launch/darknet_ros.launch](../plantbot/launch/darknet_ros.launch) line 14 to the correct launch file or you can start yolo node separatedly by one of the following command:
```
roslaunch darknet_ros yolo_v4.launch
roslaunch darknet_ros yolov4-tiny.launch
```

