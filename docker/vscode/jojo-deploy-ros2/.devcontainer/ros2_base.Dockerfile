FROM jojo/ros2:humble AS base

LABEL maintainer="1271706355@qq.com" version="1.0" description="."

# Env vars for the nvidia-container-runtime.
# 允许容器访问所有可见的 GPU
ENV NVIDIA_VISIBLE_DEVICES=all
# 指定容器可使用的 NVIDIA 驱动能力
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
# 禁用 MIT-SHM 共享内存机制（Qt 图形界面兼容 X11） 
# 多用户必须这样
# 单用户可以不用 采用映射 /dev/shm 的方式
ENV QT_X11_NO_MITSHM=1

# 宿主机 UID/GID 默认 1000；
# 容器中创建一个与宿主机 UID 相同的用户；
# 避免挂载宿主目录时文件权限变成 root。
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 让整个 Dockerfile 的 RUN 命令都用 bash + pipefail
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Find and replace any user with matching UID
# 如果镜像里已有 UID=1000 的用户（比如 ubuntu），就把它改名为 ros；
# 如果没有，就新建；
# 这样保证进入容器时能用 ros 用户，不是 root。
RUN set -eux; \
    existing_user=$(getent passwd "$USER_UID" | cut -d: -f1 || true); \
    if [ -n "$existing_user" ]; then \
        usermod -l "$USERNAME" -d "/home/$USERNAME" -m "$existing_user" && \
        groupmod -n "$USERNAME" "$existing_user" && \
        echo "Renamed $existing_user to $USERNAME" >&2; \
    else \
        groupadd --gid "$USER_GID" "$USERNAME" && \
        useradd -s /bin/bash --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME" && \
        echo "Created new user $USERNAME" >&2; \
    fi

# Ensure necessary directories and permissions
# 设置 home 和 sudo 权限
RUN mkdir -p /home/$USERNAME /run/user/$USER_UID && \
    chown -R $USER_UID:$USER_GID /home/$USERNAME /run/user/$USER_UID

# Add sudo support for the non-root user
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && touch /home/$USERNAME/.sudo_as_admin_successful

RUN cat << 'EOF' >> /home/$USERNAME/.bashrc

# CUDA
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=${CUDA_HOME}/bin:$PATH
export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${CUDA_HOME}/extras/CUPTI/lib64:$LD_LIBRARY_PATH
export C_INCLUDE_PATH=$C_INCLUDE_PATH:${CUDA_HOME}/include
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:${CUDA_HOME}/include
export LIBRARY_PATH=$LIBRARY_PATH:${CUDA_HOME}/lib64

# TensorRT
export TRT_HOME=/opt/TensorRT-10.13.3.9
export PATH=${TRT_HOME}/bin:$PATH
export LIBRARY_PATH=$LIBRARY_PATH:${TRT_HOME}/lib
export LD_LIBRARY_PATH=${TRT_HOME}/lib:$LD_LIBRARY_PATH
export C_INCLUDE_PATH=$C_INCLUDE_PATH:${TRT_HOME}/include
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:${TRT_HOME}/include

EOF