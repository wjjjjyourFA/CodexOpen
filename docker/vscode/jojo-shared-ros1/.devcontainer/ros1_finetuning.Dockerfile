FROM althack/ros2:noetic-full-amd64-base AS base

LABEL maintainer="1271706355@qq.com" version="1.0" description="."

# Set language and timezone
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#   && apt-get install locales tzdata -y --no-install-recommends \
#   && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
#   && locale-gen en_US.UTF-8 \
#   && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
#   && apt-get autoremove -y \
#   && apt-get clean -y \
#   && rm -rf /var/lib/apt/lists/*
# ENV LANG=en_US.UTF-8
# ENV DEBIAN_FRONTEND=dialog

FROM base AS update
# image is lastly updated on 2025-10-03
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#   && apt-get upgrade -y \
#   && apt-get install ros-noetic-desktop-full -y --no-install-recommends \
#   && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

### first step for some reason you know;
# 1.不是精简镜像才能这样做，否则 install wget 会失败；
# 2.在镜像中直接执行无法成功，需要进入容器后执行
# 3.小鱼一键换源；仅供参考，表明 jojo 做过什么操作；
### 配置自动交互
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get install wget ca-certificates gnupg python3 python3-yaml python3-distro -y --no-install-recommends \
#   && printf "chooses:\n" > fish_install.yaml \
#   && printf " - {choose: 5, desc: '一键配置:系统源(更换系统源,支持全版本Ubuntu系统)'}\n" >> fish_install.yaml \
#   && printf " - {choose: 2, desc: 更换系统源并清理第三方源}\n" >> fish_install.yaml \
#   && printf " - {choose: 2, desc: 根据测速结果手动选择源}\n" >> fish_install.yaml \
#   && printf " - {choose: 2, desc: ustc}\n" >> fish_install.yaml \
#   && printf " - {choose: 1, desc: 添加ROS/ROS2源}\n" >> fish_install.yaml \
#   && printf " - {choose: 1, desc: 中科大镜像源}\n" >> fish_install.yaml \
#   && printf " - {choose: y, desc: 替换/tmp/fish_install.yaml}\n" >> fish_install.yaml \
#   && wget -q --show-progress http://fishros.com/install -O fishros && /bin/bash fishros -y -f fish_install.yaml \
#   # 进行最后的清理
#   && rm -rf fish_install.yaml \
#   && apt-get clean && apt-get autoclean \
#   && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
### 手动交互
# RUN wget http://fishros.com/install -O fishros && . fishros \
  # && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up dev packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends \
    bash-completion \
    openssh-client openssh-server net-tools \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    sudo \
    # For GUI
    libx11-xcb1 libxcb1 libxcb-render0 libxcb-shm0 libxcb-xfixes0 libxcb-shape0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-util1 \
    libxcb-xinerama0 libxcb-xkb1 libxkbcommon0 libxkbcommon-x11-0 \
    libglu1-mesa libxi6 libsm6 libxrender1 libfontconfig1 \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

################
# for Windows, uncomment the following lines to 
# install WSLg dependencies and utilize vGPU acceleration 
################
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install \
#           vainfo \
#           mesa-va-drivers \
#           mesa-utils \
#    # Clean up
#         && apt-get autoremove -y \
#         && apt-get clean -y \
#         && rm -rf /var/lib/apt/lists/*
# ENV LIBVA_DRIVER_NAME=d3d12
# ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
# CMD vainfo --display drm --device /dev/dri/card0
# ENV DEBIAN_FRONTEND=dialog

################
# uncomment below for nvidia support
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
# RUN apt-get update \
#  && apt-get install -y -qq --no-install-recommends \
#   libglvnd0 \
#   libgl1 \
#   libglx0 \
#   libegl1 \
#   libxext6 \
#   libx11-6 \
# && rm -rf /var/lib/apt/lists/*

# Update some packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    # openssh-server net-tools \
    # ros-noetic-desktop \
    ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-rqt-image-view \
    # ros-noetic-ros-ign* \
    ros-noetic-rqt* \
    ignition-fortress \
    ros-dev-tools \
    # wget curl python3-pip software-properties-common \
  # && apt-get upgrade -y \
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*  
ENV DEBIAN_FRONTEND=dialog

# FROM update AS final
# 主要环境变量设置已经放在 base 阶段