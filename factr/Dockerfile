FROM nvcr.io/nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

COPY requirements.txt ./

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        build-essential \
        cmake \
        python3 \
        python3-pip \
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
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir --upgrade pip

# Python alias setup
RUN echo "alias python=python3" >> ~/.bashrc

COPY . ./

# Install Python packages
RUN python3 -m pip install --no-cache-dir -r requirements.txt \
    && cd /workspace/src/factr_teleop/factr_teleop/dynamixel \
    && python3 -m pip install -e python

CMD ["/bin/bash"]