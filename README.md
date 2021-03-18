# Naex

A package for robot navigation and exploration.
It is a work-in-progress, so this description is incomplete and may be slightly out of date.

## Nodes

### planner

The `planner` node internally builds a point map from input points clouds to assess traversability and plan paths globally.
It provides [get_plan](http://docs.ros.org/en/noetic/api/nav_msgs/html/srv/GetPlan.html) service to handle planning requests.
Both `start` and `goal` poses may be NaN.

If `start` is not provided, the plan starts with the current robot position.
Start `tolerance` in meters may be specified, which limits the distance from the requested starting position and the selected traversable starting point within the map.

If `goal` is not provided, an exploration strategy selects it, maximizing reward/cost ratio.
The reward captures visiting points from close-enough distance and prefers frontier points.
If valid goal is provided, a path is planned to the reachable point which is closest to the specified goal.
Positions of all robots are considered in assessing whether a point has been observed but robot own observation may be preferred.

The node assumes an external localization is provided.
The last request is (by default) periodically repeated and updated plan is published.

#### Subscribed topics

- `input_cloud_0` [[sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)]  
  Input point cloud from sensor 0 (remap to the actual sensor topic).
- `input_cloud_1` [sensor_msgs/PointCloud2]
- ...
- `input_map` [sensor_msgs/PointCloud2]

#### Published Topics

- `viewpoints` [sensor_msgs/PointCloud2]  
  Robot viewpoints considered in rewards.
- `other_viewpoints` [sensor_msgs/PointCloud2]  
  Viewpoints of other robots considered in rewards.
- `map` [sensor_msgs/PointCloud2]  
  Complete map used for planning, see bit field `flags` for labels.
- `updated_map` [sensor_msgs/PointCloud2]  
  Only added or removed map points (map deltas).
- `local_map` [sensor_msgs/PointCloud2]  
  Local map around the robot.
- `path` [[nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)]  
  Planned path.

        enum Flags
        {
            // Point was updated including its neighborhood. Otherwise it's queued for
            // updated.
            UPDATED     = 1 << 0,
            // A static point, not dynamic or empty, necessary for being traversable.
            STATIC      = 1 << 1,
            // Approximately horizontal orientation based on normal direction,
            // necessary condition for being traversable.
            HORIZONTAL  = 1 << 2,
            // Near another actor.
            ACTOR       = 1 << 3,
            // A point at the edge, i.e. a frontier.
            EDGE        = 1 << 4,
            // Traversable based on terrain roughness and obstacles in neighborhood.
            TRAVERSABLE = 1 << 5
        };

#### Services

- `get_plan` [nav_msgs/GetPlan]

### follower

The `follower` node follows published paths.
At each control step the closest point on the path, with an optional look-ahead distance, is selected as navigation goal.
New path is selected if the previous one has been completed or the current one has already been followed for a given time.
It stops to avoid collisions, which are assessed from input point clouds.
It can backtrack if no valid path is received for some time.

## Usage

Launchers are available, currently for SubT virtual experiments.
See the launch files for available parameters.

Launch planner (assumes localization within `subt` frame is available):

    roslaunch naex planner.launch

Launch follower (assumes localization within `subt` frame is available):

    roslaunch naex follower.launch

Launch the above with preprocessing, SLAM, recording etc.:

    roslaunch naex naex.launch

Play recorded bag files from SubT virtual robots X1, X2, X3 located inside the current working directory (topics in `/robot_data` namespace):

    bags=$(ls $(pwd)/*bag) roslaunch naex playback.launch rate:=10

