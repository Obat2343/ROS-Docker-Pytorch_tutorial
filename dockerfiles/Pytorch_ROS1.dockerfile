ARG CUDA_VER=11.8.0
ARG UBUNTU_VER=20.04
ARG TORCH_VER=2.4.1

FROM nvcr.io/nvidia/cuda:${CUDA_VER}-devel-ubuntu${UBUNTU_VER}

# メタデータ
LABEL maintainer="Takeru Oba"
LABEL description="CUDA 11.8.0 with Python, pip, and PyTorch"

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG CUDA_VER
ARG TORCH_VER
ARG ROS_DIST=noetic

# 環境変数の設定
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/usr/local/bin:${PATH}"
ENV ROS_DISTRO=${ROS_DIST}

# システムアップデートと基本ツールのインストール
RUN apt update && apt install -y --no-install-recommends \
    python3 python3-pip python3-dev python3-venv \
    build-essential git curl wget lsb-release gnupg2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# シンボリックリンクの作成
RUN ln -s /usr/bin/python3 /usr/bin/python

# CUDAバージョンの処理
RUN CUDA_MAJOR_VER=$(echo ${CUDA_VER} | cut -d. -f1-2 | sed 's/\.//g') && \
    echo "CUDA_MAJOR_VER: $CUDA_MAJOR_VER" && \
    pip install --upgrade pip && \
    pip install networkx==3.0 torch==${TORCH_VER} torchvision --index-url https://download.pytorch.org/whl/cu${CUDA_MAJOR_VER}

# ROS1リポジトリの追加
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# ROS1のインストール
RUN apt update && apt install -y ros-${ROS_DISTRO}-desktop && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ユーザー作成
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# ワークディレクトリとユーザーの設定
WORKDIR /home/${USERNAME}
USER ${USERNAME}

# ROSの設定
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# コンテナの起動スクリプト
CMD ["bash"]