# Autonomous System 2018/2019 (as1819)

This repository will bundle all required repositories to run simulations and run the actual system on the car, edited to work for Windows and Mac. Instead of running rviz natively, this solution uses VNC to run it. The goal of this is to facilitate and streamline the setup process and to make it easier for new members to start developing. For this reason this repository will also include some high level guides on how to setup the system, run it, and will also give a high level overview of all the components.

For more information refer to the READMEs of the respective submodules.

## Overview

This repository mostly includes folders and [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). Submodules are basically linked repositories inside a repository. This repository will always link to a certain state of all the submodules it refers to. Normally this should be the most-up-to-date master commit of the submodule, but when a repository updates a master this one will not automatically update with it. For now, this will manually be fixed as fast as possible (at least once a week). This slows down the development a bit, but on the other hand creates an extra opportunity to test the integration of new components into the whole system, whenever this repository updates all its submodules.

## Development

For development, we use docker. If you are new to this tool, please check the [basic tutorials](https://docs.docker.com/get-started/).

### Setup

1. Install docker

https://docs.docker.com/engine/install/ubuntu/
https://docs.docker.com/engine/install/linux-postinstall/

2. Verify docker version (need at least 19.03)

```
docker version
```

3. If you have a Nvidia GPU (**otherwise, skip this step**), install the `nvidia-container-toolkit` ([source](https://github.com/NVIDIA/nvidia-docker#ubuntu-160418042004-debian-jessiestretchbuster
))

```
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

4. Clone this repository and initialize the submodules

```
git clone git@github.com:KTHFSDV/ARCS.git --recursive --branch docker-vnc
git submodule update --init --recursive
```

5. Pull the docker image (which will also build the ROS packages in the image)

```
docker pull kthfsdv/as2021:latest
```

### Usage

During development, there is always a need to run `catkin build` to rebuild certain ROS packages.
This command is automatically run the first time you run a container, but you'll need to run it
more frequently yourself as you are making changes to your code. 

Start a container using VNC as a graphical environment:
```
make
```
You are now provided with a bash command prompt to run whatever commands you need. 

Start a container using an X server installed on your host machine:
````
make X=1
````

To run roscore:

```
make roscore
```

To run roslaunch:

```
make roslaunch ARGS="package launchfile.launch"
```

For example, to run the complete simulation:

```
make roslaunch ARGS="vehicle fs_simulation.launch"
```

Go to http://localhost:6901/ to view your graphical dekstop, where you will see simulations and applications will appear.


## Deployment - Nvidia Xavier

For deployment, we don't use docker (yet). Therefore, the setup and the workflow is slightly different (although it's based on the Dockerfile used for development). For deployment, we use a Nvidia Xavier with Ubuntu 18.04 and Jetpack 4.3, which is not yet fully compatible with docker.

### Setup

1. Install the required libraries, in case they are not there yet

```
sudo apt-get update && sudo apt-get install -y python-catkin-tools python-scipy python-pip libopencv-dev libqglviewer-headers freeglut3-dev qtbase5-dev libqglviewer-dev-qt5 doxygen libpcl-dev ros-melodic-desktop-full
```

2. To avoid most issues installing some libraries with pip (in particular scipy: see [thread](https://stackoverflow.com/questions/26575587/cant-install-scipy-through-pip)), update pip to the latest version

```
pip install --upgrade pip
```

3. Install python-related libraries with pip and upgrade numpy

```
pip install --user python-can cantools==32.20.1 rdp osqp pathlib && pip install --user --upgrade numpy
```

4. Install PyTorch for the Nvidia Xavier. Instructions are taken from [here](https://forums.developer.nvidia.com/t/pytorch-for-jetson-nano-version-1-5-0-now-available/72048). The following instructions assume Pytorch 1.4.0, Cuda 10.0, JetPack 4.3 and Python 2.7 (use the link if other setups are used):

```
wget https://nvidia.box.com/shared/static/1v2cc4ro6zvsbu0p8h6qcuaqco1qcsif.whl -O torch-1.4.0-cp27-cp27mu-linux_aarch64.whl
sudo apt-get install libopenblas-base libopenmpi-dev
pip install torch-1.4.0-cp27-cp27mu-linux_aarch64.whl
```

5. Add `export OPEN_BLAS_NUM_THREADS=1` to the .bashrc file to prevent bugs with multithreading with numpy.

6. Create a catkin workspace with catkin-tools

```
mkdir src
catkin init
cd src
```

7. Clone this repository (inside the `src/` folder) and initialize the submodules

```
git clone https://github.com/KTHFSDV/as1819.git --recursive --branch devel
git submodule update --init --recursive
```

8. Install all the ROS dependencies with rosdep

```
rosdep install --from-paths . --ignore-src -r -y
```

9. Build the ROS packages

```
catkin build -DCMAKE_BUILD_TYPE=Release
```

Additional packages:
- ```./ZED_SDK_Linux_*.run``` [download](https://download.stereolabs.com/zedsdk/3.1/jp43/jetsons) and run Zed SDK
- ```sudo apt install ros-melodic-cv-bridge ros-melodic-image-geometry ``` if purging and installing [opencv 3.4.2 for xavier](https://www.jetsonhacks.com/2018/11/08/build-opencv-3-4-on-nvidia-jetson-agx-xavier-developer-kit/) for calibration or zed

Installing OpenCV 3.4.2 (required for calibration):
- cd /tmp/
- ```git clone https://github.com/jetsonhacks/buildOpenCVXavier.git```
- cd buildOpenCVXavier
- Edit buildAndPackageOpenCV.sh and change the line OPENCV_VERSION=3.4.3 to OPENCV_VERSION=3.4.2
- ./buildAndPackageOpenCV.sh
- ./removeOpenCVSources.sh

### Usage

The commands are very similar to the docker version, with the exception that ```make``` is not used.

To rebuild ROS packages:

```
catkin build
```

To run roscore:

```
roscore
```

To run rqt:

```
rqt
```

To run rviz:

```
rviz
```

To run roslaunch:

```
roslaunch package launchfile.launch
```

For example, to run the complete simulation:

```
roslaunch vehicle fs_simulation.launch
```

To run the main launchfile for the car:

```
roslaunch vehicle run.launch
```

## Developing with this repository
When you want to get the latest changes, go at the root of this repository and use:
```git pull --recurse-submodules```


## Updating this repository
*This section is mostly relevant for the leads and software architecture members that will update this repository. Otherwise just stick to guideline above on how to use this repository easily setup your ROS workspace.*

`git submodule update --recursive --remote` to update all the submodules to the latest commit of the tracked branch
