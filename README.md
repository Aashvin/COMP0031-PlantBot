# COMP0031-PlantBot


## Requirements
<ul>
  <li>Turtlebot3: https://wiki.ros.org/turtlebot3 </li>
  <li>AMCL: http://wiki.ros.org/amcl </li>
  <li>TEB Local Planner: http://wiki.ros.org/teb_local_planner </li>
  <li>Explore Lite: http://wiki.ros.org/explore_lite </li>
  <li>YOLO: https://github.com/leggedrobotics/darknet_ros </li>
  <li>RVIZ
  <li>Gazebo
</ul>

Note when installing YOLO, make sure the COCO weights are installed.

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

## Mapping and Finding Plants Phase

### Source YOLO:

```shell
$ source <your darknet_ws>/devel/setup.bash
```

### Export Plantbot to your ROS package path:

```shell
$ cd COMP0031-PlantBot
$ export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)
$ rospack profile
```

### Export TurtleBot3 Model
```shell
$ export TURTLEBOT3_MODEL=waffle_pi
```

### Launch the mapping and finding plants phase:

```shell
$ roslaunch plantbot find_plants.launch
```

## Watering Plants Phase

### Export Plantbot to your ROS package path:

```shell
$ cd COMP0031-PlantBot
$ export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)
$ rospack profile
```

### Export TurtleBot3 Model
```shell
$ export TURTLEBOT3_MODEL=waffle_pi
```

### Run the watering plants phase:
```shell
$ roslaunch plantbot water_plants.launch
```

## Repo Organization (In directory `plantbot`)

### `launch`

+ **amcl** : the amcl node for localization
+ **move_base**: move_base for navigation stack implementation (requires TEB)
+ **navigation**: putting amcl and move_base together
+ **turtlebot3_house_pink**: launch the pink cylinder scenario in gazebo
+ **turtlebot3_slam**: launches the SLAM nodes and RVIZ for TurtleBot3
+ **plantbot_launch**: launches (plantbot) turtlebot_house3_pink, (explore_lite) explore_lite, (plantbot) move_base, (turtlebot3_slam) turtlebot3_slam
+ **darknet_ros**: YOLO for object classification
+ **pose_estimate**: pose_estimate node (for calculating plant position estimates) and coords_poller node (for receiving data raw pose data and writing the JSON and map files once mapping is done)
+ **find_plants**: launches plantbot_launch, pose_estimate
+ **water_plants**: launches (plantbot) turtlebot_house3_pink, (plantbot) move_base, (turtlebot3_bringup) turtlebot3_remote, (plantbot) amcl, (plantbot) coords_poller, saved map file from find_plants

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

#### `coord_poller/register_goal` 

Parameter: `geometry_msgs/pose`

Interface to register the pose into internal database

#### `coord_poller/poll_one`

Parameter: `std_msgs/Empty`

Bootstrap the poller or force poller to poll for next goal even if the current goal is not being fulfilled. This will be useful for exploration layer to notify the end of process.


### Parameter

#### `~min_radius` default=`1`

The minimum radius between any two goals.
