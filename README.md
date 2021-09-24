# Hi-ROS Skeleton Merger


## Dependencies
* [Hi-ROS Skeleton Messages](https://gitlab.com/hi-ros/skeleton_msgs)


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
This ROS package takes as input a SkeletonGroup estimated from multiple sources and merges the skeletons.

```
roslaunch hiros_skeleton_merger custom_configuration_example.launch
```

## Parameters

| Parameter       | Description                                                                                    |
| --------------- | ---------------------------------------------------------------------------------------------- |
| `node_required` | Set if the other ROS nodes on the PC should be killed when the driver is killed                |
| `node_name`     | Node name                                                                                      |
| `input_topic`   | Topic containing the input skeletons                                                           |
| `output_topic`  | Topic that will be published containing the merged skeletons                                   |
| `n_detectors`   | Number of detectors being used (if <= 0 the node will estimate the number of detectors online) |
| `max_delta_t`   | Maximum acceptable time delta between the skeletons to be merged (set a value < 0 to disable)  |
