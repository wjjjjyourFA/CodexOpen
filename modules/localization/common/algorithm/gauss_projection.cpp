#include "modules/localization/common/algorithm/gauss_projection.h"

namespace jojo {
namespace localization {
namespace common {

GaussProjection::GaussProjection() {};

GaussProjection::~GaussProjection() {};

void GaussProjection::SetCoordinateSystem(int coordinate_system) {
  // 根据选择的坐标系，设置不同的参数
  switch (coordinate_system) {
    case 0:  // 54年北京坐标系
      a                      = 6378245.0;
      f                      = 1.0 / 298.3;
      coordinate_system_name = "Beijing 1954";
      break;

    case 1:  // 80年西安坐标系
      a                      = 6378140.0;
      f                      = 1 / 298.257;
      coordinate_system_name = "Xi'an 1980";
      break;

    case 2:  // WGS-84坐标系
      a                      = 6378137.0;
      f                      = 1 / 298.257223563;
      coordinate_system_name = "WGS-84";
      break;

    default:
      // 默认使用 WGS-84
      a                      = 6378137.0;
      f                      = 1 / 298.257223563;
      coordinate_system_name = "WGS-84";
      break;
  }

  // 可以在这里设置一些打印信息，确保选择了正确的坐标系
  std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
  std::cerr << "++ Using coordinate system: " << coordinate_system_name
            << std::endl;
  std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
};

// 输入 ：纬度， 经度 in degrees
// 输出 ：gauss_x gauss_y in m
void GaussProjection::Forward(double longitude, double latitude, double *gauss_x,
                        double *gauss_y) {
  double longitude1, latitude1, longitude0, latitude0, X0, Y0, xval, yval;
  double e2, ee, NN, T, C, A, M;

  int ProjNo = int(longitude / ZoneWide);
  longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
  longitude0 = longitude0 * deg_to_rad;
  latitude0  = 0;

  longitude1 = longitude * deg_to_rad;  // 经度转换为弧度
  latitude1  = latitude * deg_to_rad;  // 纬度转换为弧度
  e2         = 2 * f - f * f;

  ee = e2 * (1.0 - e2);

  NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
  T  = tan(latitude1) * tan(latitude1);
  C  = ee * cos(latitude1) * cos(latitude1);

  A = (longitude1 - longitude0) * cos(latitude1);

  M = a *
      ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latitude1 -
       (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) *
           sin(2 * latitude1) +
       (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * latitude1) -
       (35 * e2 * e2 * e2 / 3072) * sin(6 * latitude1));

  xval =
      NN * (A + (1 - T + C) * A * A * A / 6 +
            (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
  yval = M + NN * tan(latitude1) *
                 (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                  (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A *
                      A * A / 720);
  X0 = 1000000L * (ProjNo + 1) + 500000L;
  Y0 = 0;

  xval = xval + X0;
  yval = yval + Y0;

  *gauss_x = xval;
  *gauss_y = yval;
}

void GaussProjection::Inverse(double gauss_x, double gauss_y, double *longitude,
                        double *latitude) {
  double longitude1, latitude1, longitude0, X0, Y0, xval, yval;
  double e1, e2, f, a, ee, NN, T, C, M, D, R, u, fai;

  int ProjNo = int(gauss_x / 1000000L);  // 查找带号
  longitude0 = (ProjNo - 1) * ZoneWide + ZoneWide / 2;
  longitude0 = longitude0 * deg_to_rad;  // 中央经线
  X0         = ProjNo * 1000000L + 500000L;
  Y0         = 0;

  xval = gauss_x - X0;
  yval = gauss_y - Y0;  // 带内大地坐标
  e2   = 2 * f - f * f;

  e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
  ee = e2 / (1 - e2);
  M  = yval;

  u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));

  fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) +
        (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u)

        + (151 * e1 * e1 * e1 / 96) * sin(6 * u) +
        (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
  C = ee * cos(fai) * cos(fai);
  T = tan(fai) * tan(fai);

  NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));

  R = a * (1 - e2) /
      sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) *
           (1 - e2 * sin(fai) * sin(fai)));
  D = xval / NN;

  // 计算经度(Longitude) 纬度(Latitude)

  longitude1 =
      longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 +
                    (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D *
                        D * D * D * D / 120) /
                       cos(fai);

  latitude1 =
      fai - (NN * tan(fai) / R) *
                (D * D / 2 -
                 (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24

                 + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) *
                       D * D * D * D * D * D / 720);  // 转换为度 DD

  *longitude = longitude1 / deg_to_rad;
  *latitude  = latitude1 / deg_to_rad;
}

// B：Latitude（纬度）——“B”源自德语 “Breite” 或法语 “Latitude”。
// L：Longitude（经度）——“L”源自德语 “Länge” 或法语 “Longitude”。
// H：Height（高程）——通常指相对于椭球的高度。
void GaussProjection::blh2xy(double blh[], double pos[]) {
  double lat = blh[0];
  double lon = blh[1];

  double x, y;
  Forward(lon, lat, &x, &y);

  pos[0] = x;
  pos[1] = y;
  pos[2] = blh[2];

  // return;
}

void GaussProjection::xy2blh(double pos[], double blh[]) {
  double x = pos[0];
  double y = pos[1];

  double lont, lat;
  Inverse(x, y, &lont, &lat);

  blh[0] = lat;
  blh[1] = lont;

  // return;
}

}  // namespace common
}  // namespace localization
}  // namespace jojo
