ARG ROS_DIST=jazzy

FROM osrf/ros:${ROS_DIST}-desktop

ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN echo "Building images based on ros:${ROS_DIST}-desktop-full" 
RUN echo "USERNAME: ${USERNAME} UID:${USER_UID} GID:${GID}" 

# ビルド用の依存関係をインストール
RUN apt update && apt install -y x11-apps less sudo

RUN groupadd -g $USER_GID $USERNAME && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    mkdir -p /home/${USERNAME}/.ros && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# ワークディレクトリとユーザーの設定
WORKDIR /home/${USERNAME}
USER ${USERNAME}

# コンテナの起動スクリプト
CMD ["bash"]