FROM althack/ros2:humble-full-amd64 AS base

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

# Set up autocompletion for user
# 自动 source ROS + workspace 环境
# 加载 vcs（版本控制工具）的补全脚本
ARG WORKSPACE
ENV WORKSPACE=${WORKSPACE}
# 每次进容器后，自动进入自己的工作空间环境，不用手动 source install/setup.bash
# RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
#   && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
RUN echo "if [ -f /usr/share/vcs2l-completion/vcs.bash ]; then source /usr/share/vcs2l-completion/vcs.bash; fi" >> /home/$USERNAME/.bashrc
  
# Set up auto-source of workspace for user
ARG ROS2WORKSPACE
ENV ROS2WORKSPACE=${ROS2WORKSPACE}
RUN echo "if [ -f ${ROS2WORKSPACE}/install/setup.bash ]; then source ${ROS2WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set up python symlinks for vscode paths
# 修复 python3 site-packages 路径
RUN PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')") \
  && tgt="/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}/site-packages" \
  && [ -d "$tgt" ] && ln -s "$tgt" "/opt/ros/${ROS_DISTRO}/lib/ros_site_packages"

# ROS2 的 ament_cmake_lint / ament_cppcheck 用的环境变量
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
