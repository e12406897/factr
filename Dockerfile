FROM nvcr.io/nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# ============================================================
# System prerequisites
# ============================================================

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    locales \
    python3 \
    python3-pip \
    python3-tk \
    git \
    build-essential \
    cmake \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Locale
# ============================================================

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ============================================================
# ROS 2 repository
# ============================================================

RUN add-apt-repository universe && \
    apt-get update && \
    export ROS_APT_SOURCE_VERSION=$( \
        curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" \
        | awk -F'"' '{print $4}' \
    ) && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# ============================================================
# ROS 2 Humble
# ============================================================

RUN apt-get update && \
    apt-get install -y \
        ros-humble-ros-base \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# rosdep
# ============================================================

RUN rosdep init 2>/dev/null || true

# ============================================================
# Additional graphical / MuJoCo dependencies
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
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
        libxkbcommon-x11-0 \
        libxcb-cursor0 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-render-util0 \
        libxcb-xinerama0 \
        libxcb-xinput0 \
        libxcb-xfixes0 \
        libxcb-randr0 \
        libxcb-shape0 \
        libxcb-sync1 \
        libxcb-xkb1 \
        libxcb-util1 \
        libx11-xcb1 \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Python dependencies
#
# requirements.txt is copied separately so that Docker can
# cache this layer when only source code changes.
# ============================================================

COPY requirements.txt /tmp/requirements.txt

RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        -r /tmp/requirements.txt

# ============================================================
# Development user
#
# UID/GID are supplied by devcontainer.json
# ============================================================

ARG USERNAME=asl_team
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd \
        --gid ${USER_GID} \
        ${USERNAME} && \
    useradd \
        --uid ${USER_UID} \
        --gid ${USER_GID} \
        --create-home \
        --shell /bin/bash \
        ${USERNAME}

# ============================================================
# User shell configuration
# ============================================================

RUN echo "source /opt/ros/humble/setup.bash" \
        >> /home/${USERNAME}/.bashrc && \
    echo "alias python=python3" \
        >> /home/${USERNAME}/.bashrc

# ============================================================
# Workspace
#
# The actual source code is mounted by devcontainer.json.
# ============================================================

WORKDIR /factr

# Make sure the development user owns its home directory
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}

CMD ["/bin/bash"]