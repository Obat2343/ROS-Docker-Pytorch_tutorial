FROM osrf/ros:iron-desktop

# 環境変数の設定
ENV DEBIAN_FRONTEND=noninteractive

# sourceからbuildするためにインストール済みROS2ライブラリを削除
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list&& \
    apt-get remove --purge -y "ros-*" && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /opt/ros/iron

# 必須パッケージのインストール
RUN apt-get update && apt-get install -y \
    python3-pip python3-setuptools \
    build-essential \
    cmake \
    git \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    wget

# 必須なPythonパッケージのインストール
RUN python3 -m pip install -U \
    rosdep \
    colcon-common-extensions \
    vcstool

# 必須なROS2ソースパッケージのクローン
ARG BUILD_DIR=/opt/ros/iron
WORKDIR ${BUILD_DIR}
RUN mkdir -p src && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src

# rosdepの初期化と更新
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && rosdep update

# 必須な依存関係のインストール
RUN rosdep install --from-paths src --ignore-src -y \
    --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    rm -rf /var/lib/apt/lists/*

# ROS2のソースビルド
RUN colcon build --symlink-install

RUN rm /ros_entrypoint.sh

# ros_entrypoint.sh を追加
COPY dockerfiles/ros_entrypoint.sh /ros_entrypoint.sh

# スクリプトを実行可能に設定
RUN chmod +x /ros_entrypoint.sh

RUN apt-get remove --purge -y "python3-catkin-pkg-modules"

# ros-core-devのインストール (ROS2のビルド後)
# Install ros-core-dev
RUN apt-get update && apt-get install -y ros-core-dev

# 好きなライブラリを入れる
RUN apt install -y less vim x11-apps sudo

# エントリーポイント
WORKDIR /opt/ros
CMD ["bash"]

# # Create and build the ros1_bridge workspace
# RUN mkdir -p /opt/ros/ros1_bridge/src
# WORKDIR /opt/ros/ros1_bridge/src
# RUN git clone https://github.com/ros2/ros1_bridge

# WORKDIR /opt/ros/ros1_bridge
# RUN . /opt/ros/iron/install/local_setup.bash
# # RUN colcon build --symlink-install

