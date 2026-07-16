FROM nvcr.io/nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install prerequisites
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    locales \
    python3-pip

# 2. Add the ROS repository
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    apt-get update && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update && \
    apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

WORKDIR /workspace

COPY . ./

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

#install python dependencies
RUN  apt update \
    && apt-get install -y --no-install-recommends \
        git \
        build-essential \
        cmake \
        python3 \
        libx11-6 \
        libx11-dev \
        libxrandr2 \
        libxinerama1 \
        libxcursor1 \
        libxi6 \
        libxxf86vm1 \
        libgl1-mesa-glx \
        libegl1 \
        libglvnd0 \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir --upgrade pip


# Python alias setup
RUN echo "alias python=python3" >> ~/.bashrc


# Install Python packages
RUN python3 -m pip install --no-cache-dir -r requirements.txt \
    && cd /workspace/src/factr_teleop/factr_teleop/dynamixel \
    && python3 -m pip install -e python

CMD ["/bin/bash"]