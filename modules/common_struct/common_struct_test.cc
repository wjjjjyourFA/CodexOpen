#include "gtest/gtest.h"

#include "modules/common_struct/basic_msgs/Pose6D.h"
#include "modules/common_struct/localization_msgs/OdometryData.h"
#include "modules/common_struct/sensor_msgs/GnssData.h"
#include "modules/common_struct/transform_msgs/Transform.h"

namespace jojo {
namespace common_struct {

TEST(CommonStructTest, DefaultPoseIsIdentity) {
  const SE3Pose pose;
  EXPECT_DOUBLE_EQ(0.0, pose.time);
  EXPECT_TRUE(pose.pos.isZero());
  EXPECT_TRUE(pose.rot.isApprox(Eigen::Quaterniond::Identity()));
  EXPECT_TRUE(pose.matrix_d().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(CommonStructTest, MatrixAndInversePreserveTime) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 3)           = 2.0;
  const SE3Pose pose     = SE3Pose::FromMatrix(matrix, 123.0);
  EXPECT_DOUBLE_EQ(123.0, pose.time);
  EXPECT_DOUBLE_EQ(123.0, pose.inverse().time);

  SE3Pose right;
  right.time = 999.0;
  EXPECT_DOUBLE_EQ(123.0, (pose * right).time);
}

TEST(CommonStructTest, SensorConversionsPropagateTimestamp) {
  GnssData gnss;
  gnss.time = 123;
  EXPECT_DOUBLE_EQ(123.0, ConvertGnssToPose(gnss).time);

  OdomData odom;
  odom.time = 456;
  EXPECT_DOUBLE_EQ(456.0, ConvertOdomToPose(odom).time);
}

TEST(CommonStructTest, TransformHeaderIsSelfContained) {
  TransformStamped transform;
  EXPECT_NE(std::string::npos, transform.ToString().find("time=0"));
}

}  // namespace common_struct
}  // namespace jojo
