#include "modules/common/math/kalman_filter_1d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace jojo {
namespace common {
namespace math {

TEST(KalmanFilter1DTest, IgnoresCorrectionBeforeInitialization) {
  KalmanFilter1D<double> filter;
  filter.Correct(10.0);
  EXPECT_FALSE(filter.IsInitialized());
  EXPECT_DOUBLE_EQ(0.0, filter.GetStateEstimate());
}

TEST(KalmanFilter1DTest, ScalarCorrectionKeepsCovarianceFinite) {
  KalmanFilter1D<double> filter;
  filter.SetStateEstimate(0.0, 1.0, 0.01, 1.0);
  filter.Correct(2.0);
  EXPECT_DOUBLE_EQ(1.0, filter.GetStateEstimate());
  EXPECT_DOUBLE_EQ(0.5, filter.GetStateCovariance());
  EXPECT_TRUE(std::isfinite(filter.GetStateCovariance()));
}

TEST(KalmanFilter1DTest, AngleCorrectionUsesShortestResidual) {
  KalmanFilter1D<double> filter;
  filter.SetStateEstimate(2.0 * M_PI - 0.1, 1.0, 0.01, 1.0);
  filter.CorrectTheta(0.1);
  EXPECT_NEAR(0.0, filter.GetStateEstimate(), 1e-12);
}

}  // namespace math
}  // namespace common
}  // namespace jojo
