# ros-RRT-planner

## Overview

The Rapidly Exploring Random Trees (RRT) algorithm, implemented in ROS to plan collision-free paths in a 2D environment.

## Usage

Run the launch file

```
roslaunch rrt_planner rrt_planner.launch
```
Which will run:
1. **RViz** to select inital position, goal location and provide visualization
2. **Map Server** which will load an image file and publish it as a `nav_msgs::OccupancyGrid` on the `/map` topic
3. **RRT Planner** to receive a map, initial pose, and goal pose, and calculate and publish a collision-free path as a `nav_msgs::Path` msg

## Config files
### Map config (`cfg/map.yaml`)

Contains parameters for the map that will be published by map_server


### RRT Planner config (`cg/config.yaml`)

Contains parameters for the RRT planner

- `RRT_show_planning` Turn intermediate visualisation on/off
- `RRT_K` Limit the number of vertices to explore 
- `RRT_timestep` Set the timestep between vertices
- `RRT_vel_max` Set a velocity limit (limits how far each vertex can explore)
- `RRT_occupied_threshold` Threshold to treat occupancy grid as occupied
- `RRT_goal_bias` Bias to select goal as the next random state

## Nodes

### map_server.py

Reads an image file and publishes it as an `nav_msgs::OccupancyGrid`

#### Published Topics
- `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) <br />
An occupancy grid representing the occupied/free space in the surroundings.

### rrt_planner

A planner decided to efficient search nonconvex spaces by building a random tree. Generates collision free paths to navigate a 2D environment.

#### Subscribed Topics
- `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) <br />
An occupancy grid of the surroundings
- `/initialpose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) <br />
A pose of the initial position
- `/move_base_simple/goal` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)) <br />
The goal location to navigate towards

#### Published Topics
- `/path` ([nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)) <br />
A series of `Pose`s which represent the planned path

## Create New Maps

Run 
```
scripts/create_map.py
```
to launch a openCV window where you can draw obstacles using the mouse.  

Press `s` to save the map as `resources/new_map.png`
