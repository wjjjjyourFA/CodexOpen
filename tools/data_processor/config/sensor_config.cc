#include "tools/data_processor/config/sensor_config.h"

std::map<string, int> CameraTypeParam = {
    {"ar0231", 1},
    {"ar0147", 1},
    {"infra", 2},
};

std::map<string, int> RadarTypeParam = {
    {"esr", 1},
    {"ars408", 2},
};

std::map<string, int> Radar4DTypeParam = {
    {"ars548", 1},
    {"hugin_arbe", 2},
};

std::map<string, int> LidarTypeParam = {
    {"m1p", 16 * 35},
    {"rs128", 2},
};
