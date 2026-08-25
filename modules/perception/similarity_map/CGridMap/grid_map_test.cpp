#include <cassert>
#include <cmath>
#include <cstdint>

#include "modules/perception/similarity_map/CGridMap/CLocalWindowMap.h"

namespace {

template <typename Map>
void Seed(Map& map) {
  for (int32_t r = 0; r < 4; ++r) {
    for (int32_t c = 0; c < 4; ++c) {
      *map.AtRC(r, c) = r * 10 + c + 1;
    }
  }
}

class InspectableLocalWindowMap : public CLocalWindowMap<int> {
 public:
  using CLocalWindowMap<int>::CLocalWindowMap;

  double residual_x() const { return residual_x_; }
  double residual_y() const { return residual_y_; }
};

void TestResidualRoundTrip() {
  CResidualGridMap<int> map(8, 8, 0.5);
  map.SetResidual(0.125, -0.125);

  const float samples[][2] = {{0.0F, 0.0F}, {0.124F, -0.124F},
                              {-0.751F, 1.249F}, {0.626F, -1.126F}};
  for (const auto& sample : samples) {
    int32_t r = 0;
    int32_t c = 0;
    assert(map.GetValueFromXY(sample[0], sample[1], r, c) != nullptr);

    float center_x = 0.0F;
    float center_y = 0.0F;
    assert(map.GetValueFromRC(r, c, center_x, center_y) != nullptr);

    int32_t round_trip_r = 0;
    int32_t round_trip_c = 0;
    assert(map.GetValueFromXY(center_x, center_y, round_trip_r,
                              round_trip_c) != nullptr);
    assert(round_trip_r == r);
    assert(round_trip_c == c);
  }
}

void TestResidualCellBoundaries() {
  CResidualGridMap<int> map(16, 16, 0.5);
  map.SetResidual(0.125, -0.125);

  constexpr float kEpsilon = 1e-4F;
  const float x_boundary = 0.125F;
  const float y_boundary = -0.125F;

  int32_t r = 0;
  int32_t c = 0;
  assert(map.GetValueFromXY(x_boundary - kEpsilon,
                            y_boundary - kEpsilon, r, c) != nullptr);
  assert(c == 7);
  assert(r == 8);

  assert(map.GetValueFromXY(x_boundary, y_boundary, r, c) != nullptr);
  assert(c == 8);
  assert(r == 7);

  assert(map.GetValueFromXY(x_boundary + kEpsilon,
                            y_boundary + kEpsilon, r, c) != nullptr);
  assert(c == 8);
  assert(r == 7);

  // Also cover a negative-index boundary.
  assert(map.GetValueFromXY(-0.375F - kEpsilon,
                            -0.625F - kEpsilon, r, c) != nullptr);
  assert(c == 6);
  assert(r == 9);
  assert(map.GetValueFromXY(-0.375F, -0.625F, r, c) != nullptr);
  assert(c == 7);
  assert(r == 8);
}

void TestWorldGridDoesNotMoveWithPoseResidual() {
  InspectableLocalWindowMap map(16, 16, 0.2);

  // Both poses are in the same world cell, but they produce different
  // pose-relative residuals. A fixed world point must still address the same
  // local cell while the rolling-window anchor is unchanged.
  assert(map.ReCenterByPose(1.01, -0.91));
  int32_t first_r = 0;
  int32_t first_c = 0;
  assert(map.GetValueFromWorldXY(1.15, -0.85, first_r, first_c) != nullptr);

  assert(!map.ReCenterByPose(1.09, -0.99));
  int32_t second_r = 0;
  int32_t second_c = 0;
  assert(map.GetValueFromWorldXY(1.15, -0.85, second_r, second_c) != nullptr);
  assert(second_r == first_r);
  assert(second_c == first_c);

  // The pose's world cell stays at the logical center until the anchor rolls.
  int32_t pose_r = 0;
  int32_t pose_c = 0;
  assert(map.GetValueFromWorldXY(1.09, -0.99, pose_r, pose_c) != nullptr);
  assert(pose_r == 7);
  assert(pose_c == 8);
}

void TestLocalAndWorldProjectionAgree() {
  InspectableLocalWindowMap map(32, 32, 0.2);
  const double pose_x = -1.13;
  const double pose_y = 2.07;
  assert(map.ReCenterByPose(pose_x, pose_y));

  const float local_samples[][2] = {
      {0.0F, 0.0F}, {0.09F, -0.09F}, {-0.21F, 0.41F},
      {0.39F, -0.61F}};
  for (const auto& local : local_samples) {
    int32_t local_r = 0;
    int32_t local_c = 0;
    assert(map.GetValueFromXY(local[0], local[1], local_r, local_c) !=
           nullptr);

    int32_t world_r = 0;
    int32_t world_c = 0;
    assert(map.GetValueFromWorldXY(pose_x + local[0], pose_y + local[1],
                                   world_r, world_c) != nullptr);
    assert(world_r == local_r);
    assert(world_c == local_c);
  }
}

void TestWorldCellBoundariesAndRoundTrip() {
  InspectableLocalWindowMap map(32, 32, 0.5);
  assert(map.ReCenterByPose(-1.1, 1.1));

  constexpr double kEpsilon = 1e-9;
  const double samples[][2] = {
      {0.0, 0.0},
      {0.5 - kEpsilon, -0.5 + kEpsilon},
      {0.5, -0.5},
      {0.5 + kEpsilon, -0.5 - kEpsilon},
      {-0.5 - kEpsilon, 0.5 + kEpsilon},
      {-0.5, 0.5},
      {-0.5 + kEpsilon, 0.5 - kEpsilon},
  };

  for (const auto& sample : samples) {
    int32_t r = 0;
    int32_t c = 0;
    assert(map.GetValueFromWorldXY(sample[0], sample[1], r, c) != nullptr);

    double center_x = 0.0;
    double center_y = 0.0;
    assert(map.GetWorldXYFromRC(r, c, center_x, center_y) != nullptr);

    int32_t round_trip_r = 0;
    int32_t round_trip_c = 0;
    assert(map.GetValueFromWorldXY(center_x, center_y, round_trip_r,
                                   round_trip_c) != nullptr);
    assert(round_trip_r == r);
    assert(round_trip_c == c);
  }
}

void TestRollingShiftContract() {
  {
    CRollingGridMap<int> map(4, 4, 1.0);
    Seed(map);
    map.Shift(1, 0);
    assert(*map.AtRC(0, 0) == 11);
    assert(*map.AtRC(3, 0) == 0);
  }
  {
    CRollingGridMap<int> map(4, 4, 1.0);
    Seed(map);
    map.Shift(-1, 0);
    assert(*map.AtRC(0, 0) == 0);
    assert(*map.AtRC(1, 0) == 1);
  }
  {
    CRollingGridMap<int> map(4, 4, 1.0);
    Seed(map);
    map.Shift(0, 1);
    assert(*map.AtRC(0, 0) == 2);
    assert(*map.AtRC(0, 3) == 0);
  }
  {
    CRollingGridMap<int> map(4, 4, 1.0);
    Seed(map);
    map.Shift(0, -1);
    assert(*map.AtRC(0, 0) == 0);
    assert(*map.AtRC(0, 1) == 1);
  }
}

void TestLargeRecenterRetainsResidual() {
  InspectableLocalWindowMap map(8, 8, 0.2);
  assert(map.ReCenterByPose(100.13, -50.07));
  assert(std::abs(map.residual_x()) > 1e-9);
  assert(std::abs(map.residual_y()) > 1e-9);

  int32_t r = 0;
  int32_t c = 0;
  assert(map.GetValueFromWorldXY(100.13, -50.07, r, c) != nullptr);
  double center_x = 0.0;
  double center_y = 0.0;
  assert(map.GetWorldXYFromRC(r, c, center_x, center_y) != nullptr);

  int32_t round_trip_r = 0;
  int32_t round_trip_c = 0;
  assert(map.GetValueFromWorldXY(center_x, center_y, round_trip_r,
                                 round_trip_c) != nullptr);
  assert(round_trip_r == r);
  assert(round_trip_c == c);
}

}  // namespace

int main() {
  TestResidualRoundTrip();
  TestResidualCellBoundaries();
  TestRollingShiftContract();
  TestWorldGridDoesNotMoveWithPoseResidual();
  TestLocalAndWorldProjectionAgree();
  TestWorldCellBoundariesAndRoundTrip();
  TestLargeRecenterRetainsResidual();
  return 0;
}
