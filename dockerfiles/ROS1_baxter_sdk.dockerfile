ARG ROS_DIST=kinetic

FROM osrf/ros:${ROS_DIST}-desktop-full

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG ROS_DIST

RUN echo "Building images based on ros:${ROS_DIST}-desktop-full" 
RUN echo "USERNAME: ${USERNAME} UID:${USER_UID} GID:${GID}" 

# ビルド用の依存関係をインストール
RUN apt update && apt install -y x11-apps less sudo

RUN groupadd -g $USER_GID $USERNAME && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    mkdir -p /home/${USERNAME}/.ros && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} && \
    usermod --shell /bin/bash ${USERNAME}

# install baxter_sdk
RUN mkdir -p ros_ws/src
WORKDIR /home/${USERNAME}/ros_ws/src
RUN git clone https://github.com/RethinkRobotics/baxter.git && \
    git clone https://github.com/RethinkRobotics/baxter_interface.git && \
    git clone https://github.com/RethinkRobotics/baxter_tools.git && \
    git clone https://github.com/RethinkRobotics/baxter_examples.git && \
    git clone https://github.com/RethinkRobotics/baxter_common.git

SHELL ["/bin/bash", "-c"]
WORKDIR /home/${USERNAME}/ros_ws
RUN source /opt/ros/${ROS_DIST}/setup.bash && \
    catkin_make && \
    catkin_make install

COPY dockerfiles/baxter.sh /home/${USERNAME}/ros_ws/baxter.sh

# ワークディレクトリとユーザーの設定
WORKDIR /home/${USERNAME}
USER ${USERNAME}

# ROSの設定
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# コンテナの起動スクリプト
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["/bin/bash"]