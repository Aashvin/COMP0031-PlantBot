# COMP0031-PlantBot

## Repo Organization

### launch:

+ **amcl** : The amcl node for localization
+ **move_base**: move_base for navigation stack implementation
+ **navigation**: putting amcl and move_base together.
+ **turtlebot3_house_pink**: launch the pink cylinder scenario in gazebo


### maps

Map files used by Rviz

### param

General configuration to the parameters.

### rviz

RViZ save file.

**Note: The current version use amcl in order to get localization data for testing the planners (TEB for local planner). Localization choice may change as project proceed.**