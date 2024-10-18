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


## Citation
Please cite the following paper:
```
Guidolin, M., Tagliapietra, L., Menegatti, E., & Reggiani, M. (2023). Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking. Computer Vision and Image Understanding, 232, 103694.
```

Bib citation source:
```bibtex
@article{GUIDOLIN2023103694,
  title = {Hi-ROS: Open-source multi-camera sensor fusion for real-time people tracking},
  journal = {Computer Vision and Image Understanding},
  volume = {232},
  pages = {103694},
  year = {2023},
  issn = {1077-3142},
  doi = {https://doi.org/10.1016/j.cviu.2023.103694},
  url = {https://www.sciencedirect.com/science/article/pii/S1077314223000747},
  author = {Mattia Guidolin and Luca Tagliapietra and Emanuele Menegatti and Monica Reggiani},
  keywords = {Markerless motion capture, Multi-view body tracking, Real-time, ROS}
}
```
