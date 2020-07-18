FROM ros:melodic as base

RUN apt-get update && apt-get install -y python-catkin-tools python-scipy python-pip \
    libopencv-dev libqglviewer-headers freeglut3-dev qtbase5-dev libqglviewer-dev-qt5 doxygen \
    ros-melodic-rviz

RUN pip install --upgrade pip

# cantools dropped support for Python2 in its latest release
RUN pip install --user python-can cantools==32.20.1 rdp osqp pathlib

RUN pip install --user --upgrade numpy

FROM base as fs_msgs
COPY ./fs_msgs/ src/fs_msgs
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as fs_simulation
COPY ./simulation/fs_simulation src/simulation/fs_simulation
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as gnss_utils
COPY ./gnss_utils/ src/gnss_utils/
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as controller
COPY ./navigation/controller/ src/navigation/controller/
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as path_planner
COPY ./navigation/path_planner/ src/navigation/path_planner
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as perception
COPY ./perception/ src/perception/
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as slam
COPY ./slam/ src/slam/
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base as vehicle
COPY ./vehicle/ src/vehicle/
RUN rosdep install --from-paths src --ignore-src -r -y


FROM base

WORKDIR /home/fs_workspace/

RUN mkdir src/

ARG ROS_PACKAGES_FOLDER=/opt/ros/melodic/

COPY --from=fs_msgs $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=fs_simulation $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=gnss_utils $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=controller $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=path_planner $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=perception $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=slam $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER
COPY --from=vehicle $ROS_PACKAGES_FOLDER $ROS_PACKAGES_FOLDER

COPY --from=fs_msgs src/fs_msgs src/fs_msgs
COPY --from=fs_simulation src/simulation/fs_simulation src/simulation/fs_simulation
COPY --from=gnss_utils src/gnss_utils src/gnss_utils
COPY --from=controller src/navigation/controller/ src/navigation/controller/
COPY --from=path_planner src/navigation/path_planner src/navigation/path_planner
COPY --from=perception src/perception src/perception
COPY --from=slam src/slam src/slam
COPY --from=vehicle src/vehicle src/vehicle

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/melodic/setup.bash && catkin config --blacklist zed_wrapper depth_clustering && catkin build
