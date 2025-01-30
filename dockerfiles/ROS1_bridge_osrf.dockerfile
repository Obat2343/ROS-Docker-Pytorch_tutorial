FROM osrf/ros:foxy-ros1-bridge

# 環境変数の設定
ENV DEBIAN_FRONTEND=noninteractive

# 期限切れのAPTキーを削除（存在しない場合でもエラー回避）
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# **APTを更新し、ROSパッケージを最新にする**
RUN apt-get update && apt-get upgrade -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# エントリーポイント
CMD ["bash"]
