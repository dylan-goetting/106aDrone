# multi-stage for setup
FROM ubuntu:jammy-20220301 as base
ARG DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    cmake \
    dirmngr \
    git \
    gnupg2 \
    libssl-dev \
    lsb-release \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    wget \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-testing.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV ROS_DISTRO rolling
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
# TODO this line doesn't get invalidated if the file itself changes
ENV ROSDISTRO_INDEX_URL https://raw.githubusercontent.com/osrf/docker_images/master/ros2/nightly/nightly/index-v4.yaml

# install python packages
RUN pip3 install -U \
    argcomplete
# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update
RUN colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml
RUN colcon metadata update

# bootstrap rosdep
RUN rosdep init

# add custom rosdep rule files
COPY prereqs.yaml /etc/ros/rosdep/
RUN echo "yaml file:///etc/ros/rosdep/prereqs.yaml" | \
    cat - /etc/ros/rosdep/sources.list.d/20-default.list > temp && \
    mv temp /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep update

# multi-stage for caching
FROM base AS cache
ARG TIMESTAMP

# install ros2 packages
RUN mkdir -p /opt/ros/$ROS_DISTRO
ARG ROS2_BINARY_URL=https://ci.ros2.org/view/packaging/job/packaging_linux/lastSuccessfulBuild/artifact/ws/ros2-package-linux-x86_64.tar.bz2
RUN wget -q $ROS2_BINARY_URL -O - | \
    tar -xj --strip-components=1 -C /opt/ros/$ROS_DISTRO

# Overwriting _colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX to point to the new install location
# Necessary because the default value is an absolute path valid only on the build machine
RUN sed -i "s|^\(_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX\s*=\s*\).*$|\1/opt/ros/$ROS_DISTRO|" \
      /opt/ros/$ROS_DISTRO/setup.sh

# copy manifests for caching
WORKDIR /opt/ros
RUN mkdir -p /tmp/opt/ros/$ROS_DISTRO && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt/ros


# multi-stage for nightly
FROM base AS ros2
ARG DEBIAN_FRONTEND=noninteractive

# install dependencies
COPY --from=cache /tmp/opt/ros/$ROS_DISTRO /opt/ros/$ROS_DISTRO
RUN apt-get update && rosdep update && rosdep install -y \
    --from-paths /opt/ros/$ROS_DISTRO/share \
    --ignore-src \
    --skip-keys " \
      cyclonedds \
      rti-connext-dds-6.0.1" \
    && rm -rf /var/lib/apt/lists/*

# install nightly
COPY --from=cache /opt/ros/$ROS_DISTRO /opt/ros/$ROS_DISTRO

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENV ROS_DOMAIN_ID=1
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM ros2 AS tello

# project deps
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install catkin_pkg rospkg av image opencv-python djitellopy2 pyyaml
# RUN apt install python3-tf*
# CPP deps
# RUN apt install ros-foxy-ament-cmake* ros-foxy-tf2* ros-foxy-rclcpp* ros-foxy-rosgraph*
# Rviz, RQT
# RUN apt install ros-foxy-rviz* ros-foxy-rqt*

WORKDIR /workspace
COPY ./vendor/github.com.tentone/tello-ros2/workspace/src ./src
# Project
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash "\
"&& rosdep install -i --from-path src "\
"&& colcon build --symlink-install --packages-select tello tello_control tello_msg"

COPY ./vendor/github.com.tentone/tello-ros2/scripts /scripts

