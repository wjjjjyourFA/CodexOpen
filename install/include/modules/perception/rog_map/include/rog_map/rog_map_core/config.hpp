/**
 * Middleware-independent ROG-Map configuration.
 *
 * Parameter-server access belongs to an adapter. This type contains only
 * algorithm values and derives the indexing/probability caches required by
 * the map implementation.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <utils/common_lib.hpp>

namespace rog_map {

#define RM_UNKNOWN_FLAG (-99999)

class Config {
 public:
  Config() = default;

  bool esdf_en{false};
  Vec3f esdf_local_update_box{5.0, 5.0, 4.0};
  double esdf_resolution{0.2};

  bool load_pcd_en{false};
  std::string pcd_name{"map.pcd"};

  double resolution{0.1};
  double inflation_resolution{0.1};
  int inflation_step{1};
  Vec3f local_update_box_d{999.0, 999.0, 999.0};
  Vec3f half_local_update_box_d{0.0, 0.0, 0.0};
  Vec3i local_update_box_i{0, 0, 0};
  Vec3i half_local_update_box_i{0, 0, 0};
  Vec3f map_size_d{10.0, 10.0, 0.0};
  Vec3f half_map_size_d{0.0, 0.0, 0.0};
  Vec3i inf_half_map_size_i{0, 0, 0};
  Vec3i half_map_size_i{0, 0, 0};
  Vec3i fro_half_map_size_i{0, 0, 0};
  double virtual_ceil_height{-0.1};
  double virtual_ground_height{-0.1};
  double safe_margin{0.0};
  int virtual_ceil_height_id_g{0};
  int virtual_ground_height_id_g{0};

  bool frontier_extraction_en{false};
  bool raycasting_en{true};
  bool unk_inflation_en{false};
  int unk_inflation_step{1};
  int intensity_thresh{-1};
  bool map_sliding_en{true};
  double map_sliding_thresh{-1.0};
  Vec3f fix_map_origin{0.0, 0.0, 0.0};

  double raycast_range_min{0.3};
  double raycast_range_max{10.0};
  double sqr_raycast_range_min{0.09};
  double sqr_raycast_range_max{100.0};
  int point_filt_num{2};
  int batch_update_size{1};
  float p_hit{0.70F};
  float p_miss{0.70F};
  float p_min{0.12F};
  float p_max{0.97F};
  float p_occ{0.80F};
  float p_free{0.30F};
  float l_hit{0.0F};
  float l_miss{0.0F};
  float l_min{0.0F};
  float l_max{0.0F};
  float l_occ{0.0F};
  float l_free{0.0F};
  double unk_thresh{0.70};

  // Used only to bound pure visualization queries; frame names, publishers,
  // and scheduling remain adapter concerns.
  Vec3f visualization_range{10.0, 10.0, 5.0};

  std::vector<Vec3i> spherical_neighbor;
  std::vector<Vec3i> unk_spherical_neighbor;

  void Finalize() {
    if (resolution <= 0.0) {
      throw std::invalid_argument("ROG-Map resolution must be positive");
    }
    if (inflation_resolution < resolution) {
      throw std::invalid_argument(
          "ROG-Map inflation_resolution must be >= resolution");
    }
    if ((map_size_d.array() <= 0.0).any()) {
      throw std::invalid_argument("ROG-Map map_size must contain 3 positives");
    }
    if (safe_margin < 0.0) {
      throw std::invalid_argument("ROG-Map safe_margin must be non-negative");
    }
    if ((local_update_box_d.array() <= 0.0).any()) {
      throw std::invalid_argument(
          "ROG-Map raycasting local_update_box must contain 3 positives");
    }
    if (esdf_en && (esdf_local_update_box.array() <= 0.0).any()) {
      throw std::invalid_argument(
          "ROG-Map ESDF local_update_box must contain 3 positives");
    }
    if (raycast_range_min < 0.0 ||
        raycast_range_max < raycast_range_min) {
      throw std::invalid_argument("ROG-Map ray_range is invalid");
    }
    if (point_filt_num <= 0) {
      point_filt_num = 1;
    }
    if (batch_update_size <= 0) {
      batch_update_size = 1;
    }
    inflation_step = std::max(0, inflation_step);
    unk_inflation_step = std::max(0, unk_inflation_step);

    resetMapSize();
    sqr_raycast_range_min = raycast_range_min * raycast_range_min;
    sqr_raycast_range_max = raycast_range_max * raycast_range_max;

    const auto logit = [](double probability) {
      if (probability <= 0.0 || probability >= 1.0) {
        throw std::invalid_argument(
            "ROG-Map probabilities must be in the open interval (0, 1)");
      }
      return static_cast<float>(
          std::log(probability / (1.0 - probability)));
    };
    l_hit = logit(p_hit);
    l_miss = logit(p_miss);
    l_min = logit(p_min);
    l_max = logit(p_max);
    l_occ = logit(p_occ);
    l_free = logit(p_free);

    spherical_neighbor.clear();
    for (int dx = -inflation_step; dx <= inflation_step; ++dx) {
      for (int dy = -inflation_step; dy <= inflation_step; ++dy) {
        for (int dz = -inflation_step; dz <= inflation_step; ++dz) {
          if (inflation_step == 1 ||
              dx * dx + dy * dy + dz * dz <=
                  inflation_step * inflation_step) {
            spherical_neighbor.emplace_back(dx, dy, dz);
          }
        }
      }
    }
    std::sort(spherical_neighbor.begin(), spherical_neighbor.end(),
              [](const Vec3i& left, const Vec3i& right) {
                return left.norm() < right.norm();
              });

    unk_spherical_neighbor.clear();
    if (unk_inflation_en) {
      for (int dx = -unk_inflation_step; dx <= unk_inflation_step; ++dx) {
        for (int dy = -unk_inflation_step; dy <= unk_inflation_step; ++dy) {
          for (int dz = -unk_inflation_step; dz <= unk_inflation_step; ++dz) {
            if (unk_inflation_step == 1 ||
                dx * dx + dy * dy + dz * dz <=
                    unk_inflation_step * unk_inflation_step) {
              unk_spherical_neighbor.emplace_back(dx, dy, dz);
            }
          }
        }
      }
      std::sort(unk_spherical_neighbor.begin(),
                unk_spherical_neighbor.end(),
                [](const Vec3i& left, const Vec3i& right) {
                  return left.norm() < right.norm();
                });
    }
  }

  void resetMapSize() {
    int inflation_ratio =
        static_cast<int>(std::ceil(inflation_resolution / resolution));
#ifdef ORIGIN_AT_CENTER
    if (inflation_ratio % 2 == 0) {
      ++inflation_ratio;
    }
#endif
    inflation_resolution = resolution * inflation_ratio;
    half_map_size_d = map_size_d / 2.0;

    const int max_step =
        unk_inflation_en ? std::max(inflation_step, unk_inflation_step)
                         : inflation_step;
    inf_half_map_size_i =
        (half_map_size_d / inflation_resolution).cast<int>() +
        (max_step + 1) * Vec3i::Ones();
    half_map_size_i =
        (inf_half_map_size_i - (max_step + 1) * Vec3i::Ones()) *
        inflation_ratio;
    if (frontier_extraction_en) {
      fro_half_map_size_i = half_map_size_i + Vec3i::Constant(1);
    }
    map_size_d =
        (half_map_size_i * 2 + Vec3i::Constant(1)).cast<double>() *
        resolution;
    half_map_size_d = map_size_d / 2.0;
    half_local_update_box_d = local_update_box_d / 2.0;
    half_local_update_box_i =
        (half_local_update_box_d / resolution).cast<int>();
    local_update_box_i =
        half_local_update_box_i * 2 + Vec3i::Constant(1);
    local_update_box_d = local_update_box_i.cast<double>() * resolution;
  }
};

}  // namespace rog_map
