FROM ros:melodic as base

RUN apt-get update && apt-get install -y python-catkin-tools python-scipy python-pip \
    libopencv-dev libqglviewer-headers freeglut3-dev qtbase5-dev libqglviewer-dev-qt5 doxygen \
    libpcl-dev ros-melodic-desktop-full

RUN pip install --upgrade pip

# cantools dropped support for Python2 in its latest release
RUN pip install --user python-can cantools==32.20.1 rdp osqp pathlib

RUN pip install --user --upgrade numpy

SHELL ["/bin/bash", "-c"]

ENV ROS_WORKSPACE=/home/fs_workspace/


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


FROM base as gnss_utils
WORKDIR $ROS_WORKSPACE
COPY ./gnss_utils src/gnss_utils/
RUN rosdep install --from-paths src --ignore-src -r -y
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
RUN catkin config --blacklist zed_wrapper
RUN source /opt/ros/melodic/setup.bash && catkin build


FROM base as slam
WORKDIR $ROS_WORKSPACE
COPY ./fs_msgs src/fs_msgs/
COPY ./slam src/slam/
RUN rosdep install --from-paths src --ignore-src -r -y
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
COPY --from=gnss_utils $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=controller $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=path_planner $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=perception $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=slam $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=vehicle $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER

WORKDIR $ROS_WORKSPACE

COPY --from=fs_msgs $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=fs_simulation $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=gnss_utils $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=controller $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=path_planner $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=perception $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=slam $ROS_WORKSPACE $ROS_WORKSPACE
COPY --from=vehicle $ROS_WORKSPACE $ROS_WORKSPACE

RUN source /opt/ros/melodic/setup.bash && catkin build

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV ROS_HOSTNAME=localhost
