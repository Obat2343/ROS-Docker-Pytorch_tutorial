ARG ROS_DIST=noetic

FROM osrf/ros:${ROS_DIST}-desktop-full

ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN echo "Building images based on ros:${ROS_DIST}-desktop-full" 
RUN echo "USERNAME: ${USERNAME} UID:${USER_UID} GID:${GID}" 

# ビルド用の依存関係をインストール
RUN apt update && apt install -y x11-apps less sudo iputils-ping net-tools mesa-utils

RUN groupadd -g $USER_GID $USERNAME && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    mkdir -p /home/${USERNAME}/.ros && \
    mkdir -p /home/${USERNAME}/demo_ws && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} && \
    usermod --shell /bin/bash ${USERNAME}

# ワークディレクトリとユーザーの設定
WORKDIR /home/${USERNAME}
USER ${USERNAME}

# ファイルのコピー
COPY dockerfiles/setup_master.sh /home/${USERNAME}/setup_master.sh

# ROSの設定
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# コンテナの起動スクリプト
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["/bin/bash"]