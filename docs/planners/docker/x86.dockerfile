##
## Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
##
## NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
## property and proprietary rights in and to this material, related
## documentation and any modifications thereto. Any use, reproduction,
## disclosure or distribution of this material and related documentation
## without an express license agreement from NVIDIA CORPORATION or
## its affiliates is strictly prohibited.
##
FROM nvidia/cuda:12.8.0-cudnn-devel-ubuntu24.04 AS torch_cuda_base

LABEL maintainer "User Name"


# Deal with getting tons of debconf messages
# See: https://github.com/phusion/baseimage-docker/issues/58
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

ENV CUDA_HOME=/usr/local/cuda
ENV PATH="${CUDA_HOME}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"

# add GL:
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libglvnd-dev \
        libgl1-mesa-dev \
        libegl1-mesa-dev \
        libgles2-mesa-dev && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Set timezone info
RUN apt-get update && apt-get install -y \
  tzdata \
  software-properties-common \
  ca-certificates \
  curl \
  gnupg \
  && rm -rf /var/lib/apt/lists/* \
  && ln -fs /usr/share/zoneinfo/Europe/Berlin /etc/localtime \
  && echo "Europe/Berlin" > /etc/timezone \
  && dpkg-reconfigure -f noninteractive tzdata \
  && mkdir -p /etc/apt/keyrings \
  && curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x6A755776" \
    | gpg --dearmor -o /etc/apt/keyrings/deadsnakes.gpg \
  && echo "deb [signed-by=/etc/apt/keyrings/deadsnakes.gpg] http://ppa.launchpadcontent.net/deadsnakes/ppa/ubuntu noble main" \
    > /etc/apt/sources.list.d/deadsnakes-ppa.list \
  && apt-get update && apt-get install -y \
  curl \
  lsb-release \
  wget \
  build-essential \
  cmake \
  git \
  git-lfs \
  iputils-ping \
  make \
  openssh-server \
  openssh-client \
  libeigen3-dev \
  libssl-dev \
  python3.11 \
  python3.11-dev \
  python3.11-venv \
  python3-pip \
  python3-ipdb \
  python3-tk \
  tcl \
  sudo git bash unattended-upgrades \
  apt-utils \
  terminator \
  glmark2 \
  && rm -rf /var/lib/apt/lists/*

RUN python3.11 -m venv /opt/venv

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

RUN python -m pip install --upgrade pip && \
    python -m pip install wstool && \
    python -m pip install -U torch==2.7.0 torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128 && \
    python -m pip install "isaacsim[all,extscache]==5.1.0" \
    --extra-index-url https://pypi.nvidia.com

# push defaults to bashrc:
RUN apt-get update && apt-get install --reinstall -y \
  libmpich-dev \
  hwloc-nox libmpich12 mpich \
  && rm -rf /var/lib/apt/lists/*

# This is required to enable mpi lib access:
ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib"



ENV TORCH_CUDA_ARCH_LIST "8.9+PTX"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Add cache date to avoid using cached layers older than this
ARG CACHE_DATE=2024-07-19

COPY entrypoint.sh /opt/entrypoint.sh
COPY patch_stdgpu.py /opt/patch_stdgpu.py
RUN chmod +x /opt/entrypoint.sh


RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# if you want to use a different version of curobo, create folder as docker/pkgs and put your
# version of curobo there. Then uncomment below line and comment the next line that clones from
# github

# COPY pkgs /pkgs

RUN mkdir /pkgs && cd /pkgs && git clone https://github.com/NVlabs/curobo.git

RUN cd /pkgs/curobo && pip3 install .[dev,usd] --no-build-isolation

WORKDIR /pkgs/curobo

# Optionally install nvblox:

# we require this environment variable to  render images in unit test curobo/tests/nvblox_test.py

ENV PYOPENGL_PLATFORM=egl

# add this file to enable EGL for rendering

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN apt-get update && \
    apt-get install -y libgoogle-glog-dev libgtest-dev curl libsqlite3-dev libbenchmark-dev && \
    cd /usr/src/googletest && cmake . && cmake --build . --target install && \
    rm -rf /var/lib/apt/lists/*

RUN sed -i -E '/THRUST_VERSION/ s@//.*$@@' "${CUDA_HOME}/include/thrust/version.h" && \
    sed -i -E '/THRUST_VERSION/ s@/\\*.*\\*/@@' "${CUDA_HOME}/include/thrust/version.h"

RUN cd /pkgs &&  git clone https://github.com/valtsblukis/nvblox.git && \
    sed -i 's#<nvToolsExt.h>#<nvtx3/nvToolsExt.h>#' /pkgs/nvblox/nvblox/include/nvblox/utils/nvtx_ranges.h && \
    sed -i 's/#include <string>/#include <array>\\n#include <string>/' /pkgs/nvblox/nvblox/include/nvblox/utils/rates.h && \
    cd nvblox && cd nvblox && mkdir build && cd build && \
    TORCH_CXX11=$(python -c "import torch; print(int(torch._C._GLIBCXX_USE_CXX11_ABI))") && \
    cmake .. -DPRE_CXX11_ABI_LINKABLE=ON -DBUILD_TESTING=OFF \
    -DCMAKE_CXX_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=${TORCH_CXX11} \
    -DCMAKE_CUDA_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=${TORCH_CXX11} && \
    python /opt/patch_stdgpu.py _deps && \
    make -j32 && \
    make install

RUN cd /pkgs && git clone https://github.com/nvlabs/nvblox_torch.git && \
    cd nvblox_torch && \
    sh install.sh $(python -c 'import torch.utils; print(torch.utils.cmake_prefix_path)') && \
    python3 -m pip install -e .

RUN python -m pip install pyrealsense2 opencv-python transforms3d

# install benchmarks:
RUN python -m pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# update ucx path: https://github.com/openucx/ucc/issues/476
RUN export LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH

RUN apt-get update && apt-get install -y --no-install-recommends curl && \
    ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F \"tag_name\" | awk -F\\\" '{print $4}')" && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-dev-tools \
      ros-jazzy-ros-base && \
    rm -rf /var/lib/apt/lists/*

RUN touch /opt/venv/lib/python3.11/site-packages/isaacsim/kit/EULA_ACCEPTED && \
    chgrp users /opt/venv/lib/python3.11/site-packages/isaacsim/kit/EULA_ACCEPTED && \
    chmod 664 /opt/venv/lib/python3.11/site-packages/isaacsim/kit/EULA_ACCEPTED


ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["/bin/bash"]
