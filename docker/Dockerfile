FROM ros:melodic as base

RUN apt-get update && apt-get install -y \
    doxygen \
    freeglut3-dev \
    libopencv-dev=3.2.0+dfsg-4ubuntu0.1 \
    libpcl-dev \
    libqglviewer-dev-qt5 \
    libqglviewer-headers \
    python-catkin-tools \
    python-scipy python-pip \
    qtbase5-dev \
    ros-melodic-rviz

RUN pip install --upgrade pip

# cantools dropped support for Python2 in its latest release
RUN pip install --user python-can cantools==32.20.1 rdp osqp pathlib pyyaml

RUN pip install --user --upgrade numpy

SHELL ["/bin/bash", "-c"]

########## VNC Setup #############
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6901 \
    VNC_RESOLUTION=1920x1080 \
    STARTUPDIR=/startup \
    SCRIPTSDIR=/install \
    NO_VNC_HOME=/root/.no_vnc

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    tigervnc-common \
    tigervnc-standalone-server \
    xfce4 \
    xfce4-goodies \
    wget
EXPOSE $VNC_PORT $NO_VNC_PORT
ADD ./docker/no_vnc.sh $SCRIPTSDIR/
RUN $SCRIPTSDIR/no_vnc.sh
###################################

ENV ROS_WORKSPACE=/home/fs_workspace/
ENV BLACKLISTED_PACKAGES="gtsam zed_nodelets zed_wrapper"


FROM base as fs_msgs
WORKDIR $ROS_WORKSPACE
COPY ./fs_msgs src/fs_msgs/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as fs_simulation
WORKDIR $ROS_WORKSPACE
COPY ./simulation src/simulation/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as sensors
WORKDIR $ROS_WORKSPACE
COPY ./sensors src/sensors/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin config --blacklist $BLACKLISTED_PACKAGES
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as controller
WORKDIR $ROS_WORKSPACE
COPY ./navigation/controller src/navigation/controller/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as path_planner
WORKDIR $ROS_WORKSPACE
COPY ./navigation/path_planner src/navigation/path_planner/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as perception
WORKDIR $ROS_WORKSPACE
COPY ./fs_msgs src/fs_msgs/
COPY ./perception src/perception/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as slam
WORKDIR $ROS_WORKSPACE
COPY ./fs_msgs src/fs_msgs/
COPY ./slam src/slam/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin config --blacklist $BLACKLISTED_PACKAGES
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as vehicle
WORKDIR $ROS_WORKSPACE
COPY ./vehicle src/vehicle/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base

ENV ROS_PACKAGES_FOLDER=/opt/ros/melodic/

COPY --from=fs_msgs $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=fs_simulation $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=sensors $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=controller $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=path_planner $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=perception $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=slam $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=vehicle $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER

WORKDIR $ROS_WORKSPACE

COPY --from=fs_msgs $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=fs_simulation $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=sensors $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=controller $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=path_planner $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=perception $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=slam $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=vehicle $ROS_WORKSPACE $ROS_WORKSPACE

RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/melodic/setup.bash && catkin build


ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV ROS_HOSTNAME=localhost

# Run VNC && noVNC
# Auto set defaults for xfce panels
RUN mkdir -p ~/.config/xfce4/xfconf/xfce-perchannel-xml \
 && cp /etc/xdg/xfce4/panel/default.xml ~/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-panel.xml

ADD ./docker/startup.sh $STARTUPDIR/
ADD ./docker/custom_vnc_lite.html $NO_VNC_HOME/vnc_lite.html
ENTRYPOINT ["/startup/startup.sh"]
