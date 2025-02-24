ARG CUDA_VER=12.4.1
ARG UBUNTU_VER=22.04

FROM nvcr.io/nvidia/cuda:${CUDA_VER}-devel-ubuntu${UBUNTU_VER}

# メタデータ
LABEL maintainer="Takeru Oba"
LABEL description="CUDA 12.4.1 with Python, pip, and PyTorch"

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG CUDA_VER
ARG TORCH_VER=2.5.1
ARG ROS_DIST=iron

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
    pip install torch==${TORCH_VER} torchvision --index-url https://download.pytorch.org/whl/cu${CUDA_MAJOR_VER}

# ROS2リポジトリの追加
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

# ROSのインストール
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