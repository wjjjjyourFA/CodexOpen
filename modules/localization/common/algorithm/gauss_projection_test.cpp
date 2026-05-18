#include <iomanip>

#include "modules/localization/common/algorithm/gauss_projection.h"

using namespace jojo::localization::common;

int main() {
  GaussProjection gp;

  // 使用 WGS-84坐标系
  gp.SetCoordinateSystem(2);

  double longitude = 112.86972392;
  double latitude  = 28.22484722;

  double gauss[2];
  gp.Forward(longitude, latitude, &gauss[0], &gauss[1]);

  std::cout << std::fixed << std::setprecision(9) << "gauss[0]: " << gauss[0]
            << " gauss[1]: " << gauss[1] << std::endl;

  return 0;
}
