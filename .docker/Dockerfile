# syntax=docker/dockerfile:experimental

FROM ros:melodic as base

ENV FS_WORKSPACE=/home/fs_workspace/
ENV CATKIN_BLACKLISTED_PACKAGES="gtsam zed_nodelets zed_wrapper"
ENV ROS_LIB_FOLDER=/opt/ros/melodic/lib
ENV ROS_HOSTNAME=localhost
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

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
ADD .docker/requirements.txt $FS_WORKSPACE
RUN pip install -r $FS_WORKSPACE/requirements.txt
RUN pip install --upgrade numpy

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
ADD .docker/no_vnc.sh $SCRIPTSDIR/
RUN $SCRIPTSDIR/no_vnc.sh

###################################

WORKDIR $FS_WORKSPACE

RUN catkin config --blacklist $CATKIN_BLACKLISTED_PACKAGES

# Run VNC && noVNC
# Auto set defaults for xfce panels
RUN mkdir -p ~/.config/xfce4/xfconf/xfce-perchannel-xml \
 && cp /etc/xdg/xfce4/panel/default.xml ~/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-panel.xml

ADD .docker/startup.sh $STARTUPDIR/
ADD .docker/custom_vnc_lite.html $NO_VNC_HOME/vnc_lite.html
RUN echo "source $/opt/ros/melodic/setup.bash 2> /dev/null || true" >> ~/.bashrc
RUN echo "source ${FS_WORKSPACE}/devel/setup.bash 2> /dev/null || true" >> ~/.bashrc

RUN --mount=target=src/,type=bind,source=. rosdep install --from-paths src/ --ignore-src -y -r

ENTRYPOINT ["/startup/startup.sh"]
