# COMP0031-PlantBot


## Requirements
Turtlebot3: https://wiki.ros.org/turtlebot3
AMCL: http://wiki.ros.org/amcl
TEB Local Planner: http://wiki.ros.org/teb_local_planner
Explore Lite: http://wiki.ros.org/explore_lite
YOLO: https://github.com/leggedrobotics/darknet_ros
RVIZ
Gazebo


## Setup

Clone the repository:

```shell
$ git clone https://github.com/Aashvin/COMP0031-PlantBot.git
```

Export the package into ROS_PACKAGE_PATH and refresh ROS package index:

```shell
$ cd COMP0031-PlantBot
$ export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)
$ rospack profile
```

## Launch Options

### Launch the entire SLAM exploration procedure:

```shell
$ roslaunch plantbot plantbot_launch.launch
```

### Launch YOLO:

Source the darknet (Make sure to install COCO weights) package setup file e.g.:

```shell
$ source darknet_ws/devel/setup.bash
```

After this, export the plantbot package into ROS_PACKAGE_PATH as shown in the steup step.

Run the darknet launch file:

```shell
$ roslaunch plantbot darknet_ros.launch
```

## Repo Organization (In directory `plantbot`)

### `launch`

+ **amcl** : the amcl node for localization
+ **move_base**: move_base for navigation stack implementation (requires TEB)
+ **navigation**: putting amcl and move_base together
+ **turtlebot3_house_pink**: launch the pink cylinder scenario in gazebo
+ **plantbot_launch**: launches (plantbot) turtlebot_house3_pink, (explore_lite) explore_lite, (plantbot) move_base, (turtlebot3_slam) turtlebot3_slam
+ **darknet_ros**: YOLO for object classification


### `maps`

Map files used by Rviz.

### `param`

General configuration to the parameters.

### `rviz`

RViZ save file.

### `launch`

Contains the launch files for the package.

### `worlds`

The world files for Gazebo.

### `worlds/models`

The custom models in use for the Gazebo worlds.

**Note: The current version use amcl in order to get localization data for testing the planners (TEB for local planner). Localization choice may change as project proceed.**


# ROS Nodes

## `plant_poses/coords_poller.py`

Polling the registered goals and send to move_base.

The next goal will be polled and applied from the cyclic queue when the current goal is in terminal state (informed by move_base).

### Subscribed Topics

Namespace: `coord_poller/`

|    topic   |  parameter |     description    |
| --- | --- | --- |
| `register_goal` | `geometry_msgs/pose` | Interface to register the pose into internal database |
| `poll_one` | `std_msgs/Empty` | Bootstrap the poller or force poller to poll for next goal even if the current goal is not being fulfilled. This will be useful for exploration layer to notify the end of process.

### Parameter

| name  | description | default |
| -- | -- | -- | 
| `~min_radius` | The minimum radius between any two goals. | `1` |

## `plant_poses/pose_estimate.py`

Estimate the world coordinates of plants from YOLO bounding box's UV coordinates

### Subscribed Topics

Namespace: `plant_pose_estimate/`

|    topic   |  parameter |     description    |
| --- | --- | --- |
|`pose` | `geometry_msgs/pose` | The estimated plant pose |
|`up`| `std_msgs/empty` | Bring up the node, used after `plant_pose_estimate/down`|
|`down`| `std_msgs/empty` | Pause the node |

### Parameters

| name | description | default |
|--|--|--|
| `~horizontal_fov` | Camera's horizontal field of view | `62.2` |
| `~img_width` | Horizontal width in pixel of captured image | `3280` |
| `~half_ray_samples` | Number of additional laser scanning ray samples in the half interval used for estimation | `1` |
| `~min_distance` | Minimum proximity distance to the estimated object (in meters) | `0.2` |
| `~converge_check` | Number of iterations that used to converge the estimation | `15` |
| `~max_scan_delta` | Maximum tolerance (in meters) on the difference between sampled scan ray, no effect if `~half_ray_samples=0` | `0.2` |