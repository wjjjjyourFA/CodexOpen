#ifndef PRIOR_MAP_LOCALIZATION_TYPES_H
#define PRIOR_MAP_LOCALIZATION_TYPES_H

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

struct LocalizationVector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct ImuData {
    double timestamp{0.0};
    LocalizationVector3 angular_velocity;
    LocalizationVector3 linear_acceleration;
};

using ImuDataConstPtr = std::shared_ptr<const ImuData>;

struct LivoxPointData {
    std::uint32_t offset_time{0};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    std::uint8_t reflectivity{0};
    std::uint8_t tag{0};
    std::uint8_t line{0};
};

using LivoxPointCloud = std::vector<LivoxPointData>;

struct Pose6D {
    double offset_time{0.0};
    std::array<double, 3> acc{{0.0, 0.0, 0.0}};
    std::array<double, 3> gyr{{0.0, 0.0, 0.0}};
    std::array<double, 3> vel{{0.0, 0.0, 0.0}};
    std::array<double, 3> pos{{0.0, 0.0, 0.0}};
    std::array<double, 9> rot{{0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0}};
};

#endif  // PRIOR_MAP_LOCALIZATION_TYPES_H
