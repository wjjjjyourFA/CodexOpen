
### 模块功能分离

我希望 经纬度 高斯坐标 和 局部坐标 分离

将 hmi localization planning 分开，
hmi 只负责经纬度，
localization 负责 经纬度和高斯 
planning 只负责 局部坐标


会不会有问题呢？

例如，无经纬度地区，如何 hmi 呢？
故人机交互设计 会影响到 planning

hmi 传来的经纬度，需要经 localization 转换成高斯坐标，再传给 planning

planning 接收 高斯坐标，并生成 局部坐标，然后规划线路

planning 传出的 局部坐标，需要经 localization 转换成经纬度，再传给 hmi

GuidePoint 使用 局部坐标系 地图，注意坐标系转换


### planning 是否添加 冻结坐标系