#!/bin/bash

# ROSマスターのIP (BaxterのIPアドレス)
ROS_MASTER_IP="011606P0021.local"
ROS_LOCAL_IP="133.21.219.152"

# 環境変数を設定
export ROS_MASTER_URI="http://$ROS_MASTER_IP:11311"
export ROS_IP="$ROS_LOCAL_IP"
export ROS_HOSTNAME="$ROS_LOCAL_IP"

# 確認メッセージ
echo "ROS_MASTER_URI is set to $ROS_MASTER_URI"
echo "ROS_IP is set to $ROS_IP"
echo "ROS_HOSTNAME is set to $ROS_HOSTNAME"

# ROSマスターにPingを送って接続確認
echo "Checking connection to ROS master..."
if ping -c 1 $ROS_MASTER_IP &> /dev/null
then
    echo "Successfully connected to ROS master at $ROS_MASTER_IP"
else
    echo "Failed to connect to ROS master at $ROS_MASTER_IP"
fi
