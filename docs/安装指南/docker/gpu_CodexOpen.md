## 1. 安装相关 CUDA 等

各位按自己的需求安装即可；
安装教程省略；

##  2. 主机测试

在主机上执行相关测试命令，检查是否已经在主机上安装成功；

<div style="display:flex; gap:16px; align-items:center;">
    <img src="./attachments/2026-06-26_112155_448.png" style="flex:none;">
    <img src="./attachments/2026-06-26_110255_029.png" style="max-width:600px; height:auto;">
</div>

注意，此处不仅是安装成功，JOJO 还将 `CUDA` 的库文件迁移到 `/path/CodexOpenExtra` 目录，目的是减少 `/根分区` 的存储压力；
有此需要的同学，可以仿照 JOJO 做法自行迁移到其他目录；

## 3. 库文件加载

- 在容器打开前，修改 `devcontainer.json` 中的相关挂载路径为 `/path/CodexOpenExtra` 目录，将物理磁盘映射到容器磁盘中；

<img src="./attachments/2026-06-26_110648_018.png" style="max-width:700px;">

- 容器打开后，将容器磁盘中的库文件，映射到**容器的相关目录**，如 `/usr/local/cuda` ，保持路径的一致性；

<img src="./attachments/2026-06-26_110747_015.png" style="max-width:700px;">

## Debug

有部分AI会说这种做法有风险，目前 JOJO 在 `ubuntu 24` 链接 `ubuntu 20` 和 `ubuntu 22` 上已经测试成功。
如果后面有同学出现该做法导致的问题，请联系 JOJO。

<img src="./attachments/20260626102132_92_260.png" style="max-width:600px;">