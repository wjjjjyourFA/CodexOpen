#include "modules/drivers/camera_tztek/driver_wrapper.h"

std::map<std::string, uint32_t> DateTypeMapValue = {
    {"RAW", 0},
    {"YUYV", 1},
    {"UYVY", 2},
    {"RAW10", 3},
};

std::map<uint32_t, const char*> DateTypeMap = {
    {0, "RAW"},  // 原始图像格式
    {1, "YUYV"},  // YUV422格式（YUV交错排列，常见于视频采集）
    {2, "UYVY"},  // YUV422格式（另一种字节排列顺序）
    {3, "RAW10"},
};