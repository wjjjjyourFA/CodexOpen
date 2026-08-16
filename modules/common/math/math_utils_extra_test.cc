#include "modules/common/math/math_utils_extra.h"

#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace jojo {
namespace common {
namespace math {

TEST(MathUtilsExtraTest, FindsNearestTimestampWithoutOverflow) {
  const std::vector<std::int64_t> timestamps{
      std::numeric_limits<std::int64_t>::min(), -10, 10,
      std::numeric_limits<std::int64_t>::max()};
  EXPECT_EQ(0, FindNearestTimestampIdx(
                   std::numeric_limits<std::int64_t>::min(), timestamps));
  EXPECT_EQ(1, FindNearestTimestampIdx(-1, timestamps));
  EXPECT_EQ(2, FindNearestTimestampIdx(1, timestamps));
  EXPECT_EQ(3, FindNearestTimestampIdx(
                   std::numeric_limits<std::int64_t>::max(), timestamps));
  EXPECT_EQ(-1, FindNearestTimestampIdx(0, {}));
}

TEST(MathUtilsExtraTest, RandomHelpersStayWithinDocumentedRange) {
  for (int i = 0; i < 1000; ++i) {
    const float signed_value = RandomFloatNeg1To1();
    const float unit_value = RandomFloat0To1();
    EXPECT_GE(signed_value, -1.0f);
    EXPECT_LE(signed_value, 1.0f);
    EXPECT_GE(unit_value, 0.0f);
    EXPECT_LE(unit_value, 1.0f);
  }
}

}  // namespace math
}  // namespace common
}  // namespace jojo
