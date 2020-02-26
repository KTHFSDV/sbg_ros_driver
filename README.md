# Autonomous System 2018/2019 (as1819)

This repository will bundle all required repositories to run simulations and run the actual system on the car. The goal of this is to facilitate and streamline the setup process and to make it easier for new members to start developing. For this reason this repository will also include some high level guides on how to setup the system, run it, and will also give a high level overview of all the components.

For more information refer to the READMEs of the respective submodules.

## Overview


## Setup and usage of this repository
This repository mostly includes folders and [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). Submodules are basically linked repositories inside a repository. This repository will always link to a certain state of all the submodules it refers to. Normally this should be the most-up-to-date master commit of the submodule, but when a repository updates a master this one will not automatically update with it. For now, this will manually be fixed as fast as possible (at least once a week). This slows down the development a bit, but on the other hand creates an extra opportunity to test the integration of new components into the whole system, whenever this repository updates all its submodules.

### Setup and usage of this repository
- For Ubuntu 18.04:
  - ```sudo apt update```
  - ```sudo apt install ros-melodic-ackermann-msgs ros-melodic-twist-mux ros-melodic-joy ros-melodic-controller-manager ros-melodic-velodyne-simulator ros-melodic-effort-controllers ros-melodic-velocity-controllers ros-melodic-joint-state-controller ros-melodic-gazebo-plugins ros-melodic-gazebo-ros-control ros-melodic-teleop-twist-keyboard ros-melodic-hector-gazebo-plugins python-scipy python-pip libopencv-dev libqglviewer-headers freeglut3-dev qtbase5-dev libqglviewer-dev-qt5 ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher ros-melodic-robot-localization ros-melodic-rqt ros-melodic-rqt-graph ros-melodic-roslint doxygen```
  - ```pip install --user python-can casadi rdp osqp```


- For Ubuntu 16.04:
  - ```sudo apt update```
  - ```sudo apt install ros-kinetic-ackermann-msgs ros-kinetic-twist-mux ros-kinetic-joy ros-kinetic-controller-manager ros-kinetic-velodyne-simulator ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-joint-state-controller ros-kinetic-gazebo-ros-control ros-kinetic-teleop-twist-keyboard ros-kinetic-hector-gazebo-plugins python-scipy python-pip libopencv-dev libqglviewer-dev freeglut3-dev qtbase5-dev ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher ros-kinetic-robot-localization ros-kinetic-rqt ros-kinetic-rqt-graph ros-kinetic-roslint doxygen```
  - ```pip install --user python-can==3.0.0 casadi rdp osqp```
  - ```pip install --user --upgrade numpy```

- If on a laptop without Cuda:
  - ```catkin config --blacklist zed_wrapper```

- (Xavier only) In the .bashrc file, add: `export OPEN_BLAS_NUM_THREADS=1`. This prevents weird bugs with multithreading with Numpy.


- ```git clone https://github.com/KTHFSDV/as1819.git --recursive --branch devel```
- ```catkin build -DCMAKE_BUILD_TYPE=Release```


Additional packages:
- ```./ZED_SDK_Linux_*.run``` [download](https://download.stereolabs.com/zedsdk/2.8/jetson_jp42) and run Zed SDK
- ```sudo apt install ros-melodic-cv-bridge ros-melodic-image-geometry ``` if purging and installing [opencv 3.4.2 for xavier](https://www.jetsonhacks.com/2018/11/08/build-opencv-3-4-on-nvidia-jetson-agx-xavier-developer-kit/) for calibration or zed

Installing OpenCV 3.4.2 (required for calibration):
- cd /tmp/
- ```git clone https://github.com/jetsonhacks/buildOpenCVXavier.git```
- cd buildOpenCVXavier
- Edit buildAndPackageOpenCV.sh and change the line OPENCV_VERSION=3.4.3 to OPENCV_VERSION=3.4.2
- ./buildAndPackageOpenCV.sh
- ./removeOpenCVSources.sh

## Building mpc files
- ```roscd mpc/resources/cvxgen_solver_sources```
- ```make clean```
- ```make```


### Developing with this repository
When you want to get the latest changes, go at the root of this repository and use:
```git pull --recurse-submodules```


### Updating this repository
*This section is mostly relevant for the leads and software architecture members that will update this repository. Otherwise just stick to guideline above on how to use this repository easily setup your ROS workspace.*

`git submodule update --recursive --remote` to update all the submodules to the latest commit of the tracked branch

## How to run (simulation)

- Running the simulation: ```roslaunch vehicle gazebo_simulation.launch```
- Keyboard control: ```roslaunch robot_control keyboard_robot_control.launch```

## How to run (car)
