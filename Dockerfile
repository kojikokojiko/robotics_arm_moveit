FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=0

# NOTE: Hash Sum mismatch 対策（開発用途のみ推奨）
RUN set -eux; \
    echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99apt-tweaks; \
    echo 'Acquire::http::Pipeline-Depth "0";' >> /etc/apt/apt.conf.d/99apt-tweaks

# 共通: apt install をリトライ付きで実行するヘルパ
RUN set -eux; \
    retry_apt_install() { \
      pkgs="$*"; \
      for i in 1 2; do \
        apt-get update; \
        if apt-get install -y --no-install-recommends --fix-missing $pkgs; then \
          return 0; \
        fi; \
        echo "apt install failed (attempt $i). retrying..."; \
        sleep 10; \
        rm -rf /var/lib/apt/lists/*; \
      done; \
      return 1; \
    }; \
    \
    # 基本ツール
    retry_apt_install \
      python3-pip \
      python3-colcon-common-extensions \
      build-essential \
      cmake \
      git \
      wget \
      vim \
      curl \
      lsb-release \
      gnupg \
      python3-rosdep; \
    \
    # ROS 2 packages (基本的なものだけ - MoveItは後で手動インストール)
    retry_apt_install \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-xacro \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers; \
    \
    rm -rf /var/lib/apt/lists/*

# Gazebo (Harmonic) repo
RUN set -eux; \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

WORKDIR /workspace

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

CMD ["/bin/bash"]
