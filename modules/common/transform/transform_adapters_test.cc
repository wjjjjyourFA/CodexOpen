#include "modules/common/transform/adapters/isam_adapter.h"
#include "modules/common/transform/adapters/loam_adapter.h"

#include "gtest/gtest.h"

TEST(TransformAdaptersTest, IsamArrayRoundTrip) {
  const isam::TransformArray input{{1.0f, -2.0f, 3.0f, 0.2f, -0.1f, 0.3f}};
  const Eigen::Matrix4f matrix = isam::Transform2Tr(input);
  const isam::TransformArray output = isam::Tr2Transform(matrix);
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_NEAR(input[i], output[i], 1e-5f);
  }
}

TEST(TransformAdaptersTest, LoamArrayRoundTrip) {
  const loam::TransformArray input{{0.1f, -0.2f, 0.3f, 1.0f, 2.0f, -3.0f}};
  const Eigen::Matrix4f matrix = loam::Transform2NormalTr(input);
  const loam::TransformArray output = loam::NormalTr2Transform(matrix);
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_NEAR(input[i], output[i], 1e-5f);
  }
}

TEST(TransformAdaptersTest, LegacyNullInputIsSafe) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Zero();
  isam::Transform2Tr(nullptr, matrix);
  EXPECT_TRUE(matrix.isIdentity());
  loam::Transform2NormalTr(nullptr, matrix);
  EXPECT_TRUE(matrix.isIdentity());
  isam::Tr2Transform(matrix, nullptr);
  loam::NormalTr2Transform(matrix, nullptr);
}
