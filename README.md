# SLAM-application: installation and test
+ (3D): [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM), [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM), [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), [Faster-LIO](https://github.com/gaoxiang12/faster-lio), [VoxelMap](https://github.com/hku-mars/VoxelMap), and [R3LIVE](https://github.com/hku-mars/r3live)
  + Tested on `Quadruped robot` in `Gazebo`
  + Tested with real-world data in E3-2 building of KAIST

<br>

## ● Results: 
### ● [`video`](https://youtu.be/RCY_q_d2Xm0): Lego-LOAM vs LIO-SAM vs LVI-SAM
### ● [`video2`](https://youtu.be/WvgGqeyHNzs): LIO-SAM vs LVI-SAM
### ● [`video3`](https://youtu.be/3d4WtK6S4Ms): LIO-SAM vs FAST-LIO2
### ● [`video4`](https://youtu.be/NmT0o268OLM): FAST-LIO2 vs Livox-mapping vs LOAM-Livox using [`Livox Mid-70 LiDAR`](https://www.livoxtech.com/mid-70), real-world
### ● [`video5`](https://youtu.be/_RgtOdK53z4): FAST-LIO2 in the building with narrow stairs using [`Ouster OS0-128`](https://ouster.com/products/scanning-lidar/os0-sensor/), real-world
### ● [`video6`](https://youtu.be/emiSJMcA8yM): FAST-LIO2 in the narrow tunnels using [`Ouster OS0-128`](https://ouster.com/products/scanning-lidar/os0-sensor/) on the UAV (drone)
### ● [`video7`](https://youtu.be/kr9Z2e8I7YQ): Faster-LIO vs FAST-LIO in the building with narrow stairs using [`Ouster OS0-128`](https://ouster.com/products/scanning-lidar/os0-sensor/), real-world

<br>

## Requirements
+ Dependencies
~~~bash
$ sudo apt-get install -y ros-melodic-navigation ros-melodic-robot-localization ros-melodic-robot-state-publisher
~~~

+ [GTSAM](https://github.com/borglab/gtsam/releases) for `LVI-SAM` and `LIO-SAM`
~~~bash
$ wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
$ unzip gtsam.zip
$ cd gtsam-4.0.2/
$ mkdir build && cd build
$ cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
$ sudo make install -j8
~~~

+ [Ceres solver](http://ceres-solver.org) only for `LVI-SAM`
~~~bash
$ sudo apt-get install -y cmake libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
$ wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
$ tar zxf ceres-solver-1.14.0.tar.gz
$ mkdir ceres-bin
$ mkdir solver && cd ceres-bin
$ cmake ../ceres-solver-1.14.0 -DEXPORT_BUILD_DIR=ON -DCMAKE_INSTALL_PREFIX="../solver"  #good for build without being root privileged and at wanted directory
$ make -j8 # 8 : number of cores
$ make test
$ make install
~~~

+ `glog`, `g++-9` and `gcc-9` for `Faster-LIO`
~~~bash
$ sudo apt-get install libgoogle-glog-dev
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt update
$ sudo apt install gcc-9 g++-9
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9
~~~

#### Note: `Ouster-ros` package cannot be built with `gcc` and `g++` with the version higher than 6
+ When building `ouster-ros`,
```bash
catkin b -DCMAKE_C_COMPILER=gcc-6 -DCMAKE_CXX_COMPILER=g++-6 -DCMAKE_BUILD_TYPE=Release
```

+ `CGAL` and `pcl-tools` for `R3LIVE`
```bash
$ sudo apt install libcgal-dev pcl-tools

Optionally,
$ sudo apt install meshlab
```

<br>

## Installation
### ● LeGO-LOAM
~~~shell
$ cd ~/your_workspace/src
$ git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~

<br>

### ● LIO-SAM
~~~shell
$ cd ~/your_workspace/src
$ git clone https://github.com/TixiaoShan/LIO-SAM.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~

<br>

### ● LVI-SAM
~~~shell
$ cd ~/your_workspace/src
$ git clone https://github.com/TixiaoShan/LVI-SAM.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~
#### ● Trouble shooting for LVI-SAM
+ for `OpenCV 4.X`, edit `LVI-SAM/src/visual_odometry/visual_loop/ThirdParty/DVision/BRIEF.cpp:53`
~~~cpp
// cv::cvtColor(image, aux, CV_RGB2GRAY);
cv::cvtColor(image, aux, cv::COLOR_RGB2GRAY);
~~~

<br>

### ● FAST-LIO2
~~~shell
$ cd ~/your_workspace/src
$ git clone https://github.com/Livox-SDK/livox_ros_driver.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release

$ cd ~/your_workspace/src
$ git clone --recursive https://github.com/hku-mars/FAST_LIO.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~

<br>

### ● Faster-LIO
~~~shell
$ cd ~/your_workspace/src
$ git clone https://github.com/gaoxiang12/faster-lio.git

$ cd faster-lio/thirdparty
$ tar -xvf tbb2018_20170726oss_lin.tgz

$ cd ~/your_workspace
$ catkin build -DCUSTOM_TBB_DIR=$(pwd)/src/faster-lio/thirdparty/tbb2018_20170726oss -DCMAKE_BUILD_TYPE=Release
~~~

<br>

### ● VoxelMap
```bash
$ cd ~/your_workspace/src
$ git clone https://github.com/Livox-SDK/livox_ros_driver.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release

$ git clone https://github.com/hku-mars/VoxelMap.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
```

#### ● Trouble shooting for VoxelMap
+ `/usr/include/lz4.h:196:57: error: conflicting declaration ‘typedef struct LZ4_stream_t LZ4_stream_t’ ...`
  + You could meet this error in `ROS-melodic`. Fix as [here](https://github.com/ethz-asl/lidar_align/issues/16#issuecomment-504348488)
~~~bash
$ sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
$ sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak

$ sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
$ sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
~~~

<br>

### ● R3LIVE
```bash
$ cd ~/your_workspace/src
$ git clone https://github.com/hku-mars/r3live.git
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
```

#### ● How to properly set configuration for R3LIVE
+ Camera calibration - use `Kalibr` or `camera_calibration`
  + `Kalibr` - refer original [repo](https://github.com/ethz-asl/kalibr)
  + `camera_calibration` - use as [here](https://github.com/heethesh/lidar_camera_calibration)
+ Lidar-Camera calibration
  + Other spinning LiDARs are not supported yet (for RGB mapping), but try to use `lidar_camera_calibration` [repo](https://github.com/heethesh/lidar_camera_calibration), if you want to.
  + For `LiVOX` LiDAR, use `livox_camera_calib` [repo](https://github.com/hku-mars/livox_camera_calib)
    + Record a `bag file` of LiVOX LiDAR data and capture the image from `RGB camera` you use.
    + Convert a `bag file` into a `PCD file` with (change directories in the launch file):
    ```bash
    $ roslaunch livox_camera_calib bag_to_pcd.launch
    ```
    + Then, calibrate LiDAR and camera as (change directories in the launch and config files):
    ```bash
    $ roslaunch livox_camera_calib calib.launch
    ```
##### **Note**: extrinsic rotational parameter from `livox_camera_calib` should be transposed in the `r3live_config.yaml` file. Refer my [extrinsic result](r3live/extrinsic.txt) and [r3live config file](r3live/r3live_config.yaml)

<p align="center">
  <img src="r3live/0.png" width="300"/>
  <img src="r3live/pcd.png" width="300"/>
  <br>
  <em>Left: Target image. Right: Target PCD</em>
</p>
<p align="center">
  <img src="r3live/calib.png" width="300"/>
  <img src="r3live/calib2.png" width="300"/>
  <br>
  <em>Left: calibrated image and residuals. Right: calibrated image</em>
</p>

<br>

## How to run
#### ● check each of config files and launch files in the folders of this repo

#### Trouble shooting for [`Gazebo Velodyne plugin`](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/)
+ When using `CPU ray`, instead of `GPU ray`, height - width should be interchanged, I used [this script file](https://github.com/engcang/SLAM-application/blob/main/lidar_repair.py)
