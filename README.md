# multisensor-SLAM

## Software installation

Software dependencies and installation procedure are specified in the "setup/installation"-folder.

## Download KITTI odometry data

Download either only the 00 sequence of the kitti dataset, or the entire odometry dataset with all sequences.
```bash
$ cd setup/download
$ ./download-kittidata-00-full
```


### Create rosbag from KITTI data

```bash
$ cd ~/Videos
$ kitti2bag -t 2011_10_03 -r 0027 raw_sync .
```


## Setup

```bash
$ git clone https://github.com/tshellum/multisensor-SLAM.git
$ cd multisensor-SLAM
$ git submodule update --init --recursive
$ cd setup/build
$ ./build_DLoopDetector
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


## Visualization

### Plotting the optimization graph

Run the system following the instructions above. Convert the created .dot file to a visual file using the following command.

```bash
dot -Tps graph.dot -o graph.ps
```

### Plotting the estimated trajectory compared to GNSS data

```bash
$ rosrun rpg_trajectory_evaluation analyze_trajectory_single.py results/ --recalculate_errors
```