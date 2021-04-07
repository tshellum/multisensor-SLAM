# Setup

In this folder you will find scripts for installing the nessecary software dependencies for the multisensor-SLAM system. 
The scripts are extentions of the [scripts](https://github.com/tek5030/setup_scripts) supplied in the course TTK21 at NTNU.

## Prerequesites
- Ubuntu 18.04

### Installed project dependencies
The software is tested using on the versions listed below.

- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [OpenCV >= 3.2.0](https://opencv.org/)
- [GTSAM >= 4.1.0](https://gtsam.org/)
- [Point Cloud Library (PCL) >= 1.8.1](https://pointclouds.org/downloads/)
- [Eigen3 >= ...](http://eigen.tuxfamily.org/)
- [Sophus >= ...](https://github.com/strasdat/Sophus)
- [Boost >= ...](https://www.boost.org/)
- [CMake >= ...](https://cmake.org/download/)

Notice that the software versions that are used are upper bound by   
[the ROS Melodic software support](https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023 "Dependencies for ROS melodic").


## Installation procedure

1. Install git.
```bash
$ sudo apt install git-all
$ git config --global user.name "Your name here"
$ git config --global user.email "your_email@example.com"
```

2. In "run-complete-setup" include the software packages that are not previously installed. Notice that dependencies also need to be met; These may be found on the pages listed above.
    1. Pangolin are not used and therefore not neccessary to install. 
    2. The 00-install-basics script only downloads basic software that is not included with ubuntu and are more relevant for developement and testing of the software.

3. Execute the "run-complete-setup" script. If the scripts are not allready executable, run chmod +x. 
```bash
$ ./run-complete-setup
```

The software will be downloaded to a seperate folder at ~/home/software. 



### Convert KITTI raw data into ROS bags

The [KITTI odometry dataset](http://www.cvlibs.net/datasets/kitti/index.php) is used for testing purposes. In order of transforming the raw data into rosbags, either [kitti2bag](https://github.com/tomas789/kitti2bag) or [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) may be used. 


#### kitti2bag

Either follow the installation instructions of the [main kitti2bag repository](https://github.com/tomas789/kitti2bag) or the [kitti2bag fork by alteretch](https://github.com/alteretch/kitti2bag). [Issues](https://github.com/tomas789/kitti2bag/issues/18) were experienced with the main repo and this fork worked well. However, for both repos the inertial coordinate frame does not follow [the frame of the GNSS](http://www.cvlibs.net/datasets/kitti/setup.php).

Alternative installation guides like [this](https://idorobotics.com/2019/05/29/converting-the-kitti-dataset-to-rosbags/) or [this](https://www.gitmemory.com/issue/tomas789/kitti2bag/10/479454686) provide instructions on how to download kitti2bag, and then download and convert individual sequences. Ground truth data need to be downloaded and added manually by requesting access: http://www.cvlibs.net/datasets/kitti/eval_odometry.php. 


#### kitti_to_rosbag

Not yet tested!! Clone the repository https://github.com/tshellum/kitti2rosbag and build it. This repo has added [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) and all [specified dependencies](https://www.programmersought.com/article/39825817539/). 

```bash
$ git clone https://github.com/tshellum/kitti2rosbag.git
$ cd kitti2rosbag && git submodule update --init --recursive
$ catkin build
```


