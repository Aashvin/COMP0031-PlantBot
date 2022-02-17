# COMP0031-PlantBot

## Setup

first export current directory into ROS_PACKAGE_PATH

```shell
$ export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)
```

then run the refresh the ros package index:

```shell
$ rospack profile
```

and launch it like other ros package! for example:

```shell
$ roslaunch plantbot turtlebot3_house_pink.launch
```

## Repo Organization (In directory `plantbot`)

### `launch`

+ **amcl** : The amcl node for localization
+ **move_base**: move_base for navigation stack implementation
+ **navigation**: putting amcl and move_base together.
+ **turtlebot3_house_pink**: launch the pink cylinder scenario in gazebo


### `maps`

Map files used by Rviz

### `param`

General configuration to the parameters.

### `rviz`

RViZ save file.

**Note: The current version use amcl in order to get localization data for testing the planners (TEB for local planner). Localization choice may change as project proceed.**