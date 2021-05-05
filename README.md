# multisensor-SLAM

## Software installation

Software dependencies and installation procedure are specified in the "setup/installation"-folder.

## Download KITTI data and generate rosbag

Download only the 00 sequence of the kitti dataset. The script also generates a rosbag from the kitti data.
```bash
$ cd setup/download/
$ ./download-kittidata-00-full
```


## Setup

```bash
$ git clone https://github.com/tshellum/multisensor-SLAM.git
$ cd multisensor-SLAM
$ git submodule update --init --recursive
$ cd setup/
$ ./uncompress_vocabulary.sh
```


## Usage

Modify the config files to fit the dataset that is to be used. Then run:

```bash
$ roscore
$ catkin build
$ roslaunch vo kitti.launch
$ roslaunch backend kitti.launch
```

To visualize the generated point cloud and the motion of the vessel, type:

```bash
$ roslaunch cloud_viewer viz.launch
```

To save the motion of the vessel onto a txt file, type:

```bash
$ roslaunch motion2file eval.launch
```

Then play the rosbag

```bash
$ rosbag play /path/to/rosbag
```

Example: For the 00 sequence of the rosbag it is recommended to play from 3 seconds in becasue the measurement rate is for some reason lower initially. 
```bash
$ rosbag play -s 3 ~/Videos/kitti/kitti_2011_10_03_drive_0027_sync.bag
```

## Visualization

### Plotting the optimization graph

Run the system following the instructions above. Convert the created .dot file to a visual file using the following command.

```bash
dot -Tps graph.dot -o graph.ps
```

### Plotting the estimated trajectory compared to GNSS data

This module also computes the ATE and RTE for the odometry vs the ground truth data based on the closest timestamps.
```bash
$ rosrun rpg_trajectory_evaluation analyze_trajectory_single.py results/ --recalculate_errors
```