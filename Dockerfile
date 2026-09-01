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
# ROS 2 Humble - July 2023 snapshot
# ============================================================

RUN add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl gnupg ca-certificates && \
    apt-key adv \
        --keyserver hkp://keyserver.ubuntu.com:80 \
        --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
    echo "deb http://snapshots.ros.org/humble/2023-07-24/ubuntu \
        $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list

# ============================================================
# ROS 2 Humble + workspace dependencies
# ============================================================

RUN apt-get update && \
    apt-get install -y \
        ros-humble-ros-base \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rviz2 \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-teleop-twist-keyboard \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        ros-humble-sick-safetyscanners2 \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-ros2controlcli \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-moveit-msgs \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-pinocchio \
        ros-humble-ros2-control-test-assets \
        ros-humble-diff-drive-controller \
        python3-requests \
        libpoco-dev \
        libgtest-dev \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep \
        iproute2 \
        iputils-ping \
        netcat-openbsd && \
        rm -rf /var/lib/apt/lists/*

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
ENV USERNAME=${USERNAME}

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
# gosu: lets the entrypoint do root-only setup (USB latency timer)
# then cleanly drop privileges to ${USERNAME} for the actual session.
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends gosu && \
    rm -rf /var/lib/apt/lists/*

# ============================================================
# User shell configuration
# ============================================================
    
RUN echo "source /opt/ros/humble/setup.bash"\
        >> /home/${USERNAME}/.bashrc && \
    echo 'source /factr/install/setup.bash' \
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

# ============================================================
# Entrypoint
#
# Stays root at container start (no final USER switch here) so the
# entrypoint can do root-only setup (USB latency timer) before dropping
# privileges to ${USERNAME} via gosu for the actual session. VS Code's
# own terminals/tasks still attach as devcontainer.json's `remoteUser`,
# independent of this.
# ============================================================

COPY .devcontainer/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]