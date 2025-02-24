FROM ubuntu:22.04

# 非対話モードを設定
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=iron

# 基本的なツールをインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3 \
    python3-pip \
    libssl-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    libtinyxml2-dev \
    zlib1g-dev \
    libasio-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# タイムゾーン設定と必要なパッケージのインストール
RUN apt-get update && apt-get install -y tzdata && \
    echo "Asia/Tokyo" > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata

RUN apt update && apt install -y \
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
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-setuptools \
    python3-rosdep2 \
    wget

# Install colcon from PyPI, rather than apt packages
RUN python3 -m pip install -U colcon-common-extensions vcstool

ARG BUILD_DIR=/opt/ros
WORKDIR ${BUILD_DIR}
RUN mkdir -p src && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src

# rosdepの初期化と更新
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && rosdep update

RUN rosdep install --from-paths src --ignore-src -y --rosdistro iron\
    --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-math6 ignition-cmake2" && \
    rm -rf /var/lib/apt/lists/*

# ROS2のソースビルド
RUN colcon build

# ros-core-devのインストール (ROS2のビルド後)
RUN apt-get update && apt install -y ros-core-dev && rm -rf /var/lib/apt/lists/*