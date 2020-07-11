FROM ros:melodic

RUN apt-get update && apt-get install -y python-catkin-tools python-scipy python-pip \
    libopencv-dev libqglviewer-headers freeglut3-dev qtbase5-dev libqglviewer-dev-qt5 doxygen \
    ros-melodic-rviz

RUN pip install --upgrade pip

# cantools dropped support for Python2 in its latest release
RUN pip install --user python-can cantools==32.20.1 rdp osqp pathlib

RUN pip install --user --upgrade numpy

WORKDIR /home/fs_workspace/

RUN mkdir src/

COPY ./fs_msgs/ src/fs_msgs

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./simulation/fs_simulation src/simulation/fs_simulation

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./gnss_utils/ src/gnss_utils/

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./navigation/controller/ src/navigation/controller/

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./navigation/path_planner/ src/navigation/path_planner

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./perception/ src/perception/

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./simulation/fs_simulation src/simulation/fs_simulation

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./slam/ src/slam/

RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./vehicle/ src/vehicle/

RUN rosdep install --from-paths src --ignore-src -r -y

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/melodic/setup.bash && catkin config --blacklist zed_wrapper depth_clustering && catkin build
