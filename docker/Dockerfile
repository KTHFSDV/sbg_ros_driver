# syntax=docker/dockerfile:experimental

FROM ros:melodic as base

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    doxygen \
    freeglut3-dev \
    libopencv-dev=3.2.0+dfsg-4ubuntu0.1 \
    libpcl-dev \
    libqglviewer-dev-qt5 \
    libqglviewer-headers \
    python-catkin-tools \
    python-scipy python-pip \
    qtbase5-dev \
    vim \
    ros-melodic-rviz

RUN pip install --upgrade pip

# cantools dropped support for Python2 in its latest release
RUN pip install --user python-can cantools==32.20.1 rdp osqp==0.6.1 pathlib pyyaml

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
ENV ROS_LIB_FOLDER=/opt/ros/melodic/lib
ENV ROS_HOSTNAME=localhost
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR $ROS_WORKSPACE

RUN catkin config --blacklist $BLACKLISTED_PACKAGES

# Run VNC && noVNC
# Auto set defaults for xfce panels
RUN mkdir -p ~/.config/xfce4/xfconf/xfce-perchannel-xml \
 && cp /etc/xdg/xfce4/panel/default.xml ~/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-panel.xml

ADD ./docker/startup.sh $STARTUPDIR/
ADD ./docker/custom_vnc_lite.html $NO_VNC_HOME/vnc_lite.html

RUN --mount=target=src/,type=bind,source=. rosdep install --from-paths src/ --ignore-src -y -r

ENTRYPOINT ["/startup/startup.sh"]
