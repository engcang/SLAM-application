# SLAM-application: installation and application
+ (3D): [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM), [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), and [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
  + on `Quadruped robot` in `Gazebo`

<br>

## Installation
### LIO-SAM
+ Dependencies
~~~
$ sudo apt-get install -y ros-melodic-navigation ros-melodic-robot-localization ros-melodic-robot-state-publisher
~~~
+ [GTSAM](https://github.com/borglab/gtsam/releases)
~~~
$ wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
$ unzip gtsam.zip
$ cd gtsam-4.0.2/
$ mkdir build && cd build
$ cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
$ sudo make install -j8
~~~
+ Install
~~~
$ cd ~/your_workspace/src
$ git clone https://github.com/TixiaoShan/LIO-SAM.git
$ cd ..
$ catkin build
~~~

<br>

## How to run
### LIO-SAM in Gazebo
#### with `[Gazebo Velodyne plugin](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/)`
+ height - width should be interchanged, I used [this script file]()
