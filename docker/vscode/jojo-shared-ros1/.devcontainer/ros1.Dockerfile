#############################################
# Created from template ros.dockerfile.jinja
#############################################

###########################################
# Base image
###########################################
ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE} AS base
LABEL maintainer="1271706355@qq.com" version="1.0" description="."

# -----------------------------
# 替换 apt 为国内源 + 设置时区
# -----------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN sed -i 's@archive.ubuntu.com@mirrors.ustc.edu.cn@g' /etc/apt/sources.list \
  && sed -i 's@security.ubuntu.com@mirrors.ustc.edu.cn@g' /etc/apt/sources.list \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
     software-properties-common \
     locales \
     tzdata \
     ca-certificates \
     apt-transport-https \
     curl \
     wget \
     dirmngr \
     gnupg2 \
     lsb-release \
     sudo \
  && add-apt-repository universe \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
     fonts-noto-cjk \
     fonts-wqy-zenhei \
     fonts-wqy-microhei \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
  && echo "Asia/Shanghai" > /etc/timezone \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=

###########################################
# Develop image
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
# Setup ROS1 environment
ENV ROS_DISTRO=noetic
ENV ROS_VERSION=1
ENV ROS_PYTHON_VERSION=3

ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros

ENV PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
ENV PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic

ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV DEBIAN_FRONTEND=

################
# Expose the nvidia driver to allow opengl
# Dependencies for glvnd and X11.
################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
     libglvnd0 \
     libgl1 \
     libglx0 \
     libegl1 \
     libxext6 \
     libx11-6 \
  && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
# 允许容器访问所有可见的 GPU
ENV NVIDIA_VISIBLE_DEVICES=all
# 指定容器可使用的 NVIDIA 驱动能力
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
# 禁用 MIT-SHM 共享内存机制（Qt 图形界面兼容 X11） 
# 多用户必须这样
# 单用户可以不用 采用映射 /dev/shm 的方式
ENV QT_X11_NO_MITSHM=1
ENV DEBIAN_FRONTEND=

###########################################
# User image
###########################################
FROM full AS user

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
     cmake ninja-build \
     libboost-all-dev \
     # protobuf-compiler libprotobuf-dev \
     protobuf-compiler=3.6.1.* libprotobuf-dev=3.6.1.* \
     libprotoc-dev \
     libgoogle-perftools-dev google-perftools \
     libyaml-cpp-dev \
     libpcap-dev \
     libepoxy-dev \
     libfmt-dev \
     libgflags-dev \
     libgoogle-glog-dev \
     libgtest-dev libgmock-dev \
     nlohmann-json3-dev \
     libtinyxml2-dev \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

###########################################
# Final stage image
###########################################
FROM user AS finetuning

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
     build-essential \
     bash-completion \
     openssh-client openssh-server \
     git \
     # For GUI / Qt / RViz
     libx11-xcb1 libxcb1 libxcb-render0 libxcb-shm0 libxcb-xfixes0 \
     libxcb-xinerama0 libxcb-xkb1 libxkbcommon0 libxkbcommon-x11-0 \
     libglu1-mesa libxi6 libsm6 libxrender1 libfontconfig1 libxcb-randr0 \
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=