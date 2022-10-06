# Hi-ROS Skeleton Merger

This ROS package takes as input the SkeletonGroup published by the [Skeleton Tracker](https://github.com/hiros-unipd/skeleton_tracker) node and merges the skeletons estimated from different sources.


## Dependencies
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)
* [Hi-ROS Skeleton Tracker](https://github.com/hiros-unipd/skeleton_tracker)


## Parameters
| Parameter               | Description                                                                                       |
| ----------------------- | ------------------------------------------------------------------------------------------------- |
| `input_topic`           | Input topic containing the tracked skeletons                                                      |
| `output_topic`          | Output topic containing the merged skeletons                                                      |
| `n_detectors`           | Number of detectors being used (if <= 0 the node will estimate the number of detectors online)    |
| `max_delta_t`           | Maximum acceptable time delta between the skeletons to be merged (set a value < 0 to disable)     |
| `max_position_delta`    | Maximum position delta not to consider a marker as an outlier                                     |
| `max_orientation_delta` | Maximum orientation delta not to consider a link as an outlier                                    |
| `pelvis_marker_id`      | ID of the pelvis marker, used to identify when a detector estimated a pose flipped by 180 degrees |


## Usage
```
ros2 launch hiros_skeleton_merger default.launch.py
```
