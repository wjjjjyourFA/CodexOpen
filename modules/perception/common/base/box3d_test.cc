#include "modules/perception/common/base/box3d.h"

#include "gtest/gtest.h"

namespace jojo {
namespace perception {
namespace base {

TEST(LidarCoreTest, operator_test) {
  {
    BBox3D<int> bbox(1, 2, 3, 4, 5, 6);
    Cube<int> cube = static_cast<Cube<int>>(bbox);
    EXPECT_EQ(cube.x, 1);
    EXPECT_EQ(cube.y, 2);
    EXPECT_EQ(cube.z, 3);
    EXPECT_EQ(cube.length, 3);
    EXPECT_EQ(cube.width, 3);
    EXPECT_EQ(cube.height, 3);
  }
  {
    Cube<int> cube(1, 2, 3, 4, 5, 6);
    BBox3D<int> bbox = static_cast<BBox3D<int>>(cube);
    EXPECT_EQ(bbox.xmin, 1);
    EXPECT_EQ(bbox.ymin, 2);
    EXPECT_EQ(bbox.zmin, 3);
    EXPECT_EQ(bbox.xmax, 5);
    EXPECT_EQ(bbox.ymax, 7);
    EXPECT_EQ(bbox.zmax, 9);
  }
}

}  // namespace base
}  // namespace perception
}  // namespace jojo