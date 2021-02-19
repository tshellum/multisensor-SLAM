# multisensor-SLAM

## Software installation

Software dependencies and installation procedure are specified in the "Setup"-folder.

## Download KITTI odometry data

Download either only the 00 sequence of the kitti dataset, or the entire odometry dataset with all sequences.
```bash
$ cd download
$ ./download-kittidata-00-full
or
$ ./download-kittidata-odometry
```


### Create rosbag from KITTI data

```bash
$ cd ~/Videos
$ kitti2bag -t 2011_10_03 -r 0027 raw_sync .
```

or 

```bash
$ cd ~/Videos/dataset
$ sudo kitti2bag odom -s 00
```


## Setup

```bash
$ git clone https://github.com/tshellum/multisensor-SLAM.git
$ cd multisensor-SLAM
$ git submodule update --init --recursive
```


## Usage

Modify the config files to fit the dataset that is to be used.

```bash
$ roscore
$ catkin build
$ roslaunch stereo_frontend kitti.launch
$ roslaunch backend kitti.launch
$ roslaunch cloud_viewer viz.launch
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