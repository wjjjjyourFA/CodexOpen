#ifndef TRANSFORM_UTILS_HPP
#define TRANSFORM_UTILS_HPP




// 空间转换函数
namespace Trans_Our {


// 车上的pose数据
// 所有都是 右前上转右前上
/*
 * pos： 右前上
 * Tr ： 右前上
*/
template <typename T1, typename T2>
static void Euler2Rt(Pose_txyzrpy pos, T1 &R, T2 &t) {

    R.setIdentity();
    t.setZero();

    T1 Rx, Ry, Rz;
    double crz = cos(pos.yaw);
    double srz = sin(pos.yaw);
    double crx = cos(pos.pitch);
    double srx = sin(pos.pitch);
    double cry = cos(pos.roll);
    double sry = sin(pos.roll);

    Ry<<    cry, 0, sry,
            0 , 1, 0,
            -sry, 0, cry;
    Rx<<    1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<    crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R = Rz*Rx*Ry; // R_azimuth * R_pitch * R_roll

    t<< pos.x,  pos.y,  pos.z;

}
template <typename T>
static void Euler2Tr(Pose_txyzrpy pos, T &Tr) {
    //    T Tr;
    Tr.setIdentity();

    double eulerZ = pos.yaw;
    double eulerY = pos.roll;
    double eulerX = pos.pitch;

    double cx = cos(eulerX);
    double cy = cos(eulerY);
    double cz = cos(eulerZ);
    double sx = sin(eulerX);
    double sy = sin(eulerY);
    double sz = sin(eulerZ);
    double cc = cy * cz;
    double cs = cy * sz;
    double sc = sy * cz;
    double ss = sy * sz;

    Tr(0,0) = cc - sx * ss;     Tr(0,1) =-cx * sz;      Tr(0,2) = sc + sx * cs;
    Tr(1,0) = cs + sx * sc;     Tr(1,1) = cx * cz;      Tr(1,2) = ss - sx * cc;
    Tr(2,0) =-cx * sy;          Tr(2,1) = sx;           Tr(2,2) = cx * cy;

    Tr(0,3) = pos.x,    Tr(1,3) = pos.y,        Tr(2,3) = pos.z;


    //    return Tr;
}
template <typename T>
static void Euler2R(Pose_txyzrpy pos, T &R) {

    R.setIdentity();

    double eulerZ = pos.yaw;
    double eulerY = pos.roll;
    double eulerX = pos.pitch;

    double cx = cos(eulerX);
    double cy = cos(eulerY);
    double cz = cos(eulerZ);
    double sx = sin(eulerX);
    double sy = sin(eulerY);
    double sz = sin(eulerZ);
    double cc = cy * cz;
    double cs = cy * sz;
    double sc = sy * cz;
    double ss = sy * sz;

    R(0,0) = cc - sx * ss;     R(0,1) =-cx * sz;      R(0,2) = sc + sx * cs;
    R(1,0) = cs + sx * sc;     R(1,1) = cx * cz;      R(1,2) = ss - sx * cc;
    R(2,0) =-cx * sy;          R(2,1) = sx;           R(2,2) = cx * cy;

//    return R;
}

template <typename T>
static void Tr2Euler(Pose_txyzrpy &pos,  T Tr) {
    pos.x = Tr(0,3),            pos.y = Tr(1,3),        pos.z = Tr(2,3);
    pos.pitch = asin(Tr(2,1));
    double cx = cos(pos.pitch);
    pos.roll = -atan2(Tr(2,0)/cx, Tr(2,2)/cx);
    pos.yaw  = -atan2(Tr(0,1)/cx, Tr(1,1)/cx);
}
}



namespace Trans_Ros {

/*
 * pos： 右前上
 * Tr ： 前左上
*/
static void GetT_FromEuler_handler(Pose_txyzrpy pos, Eigen::Matrix4d &Tr) {

    Eigen::Affine3d affine;
    pcl::getTransformation(pos.x, pos.y, pos.z, pos.roll, -pos.pitch, pos.yaw+M_PI/2, affine);
    Tr = affine.matrix();

}

/*
 * Tr ： 前左上
 * pos： 右前上
*/
static void GetEuler_FromT_handler(Pose_txyzrpy &pos, Eigen::Matrix4d  Tr) {
    Eigen::Transform<double, 3, Eigen::Affine> affine (Tr);
    double x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles (affine, x, y, z, roll, pitch, yaw );

    pos.x = x;          pos.y = y;              pos.z = z;
    pos.pitch = -pitch;      pos.roll = roll;   pos.yaw = yaw>M_PI/2 ? yaw-M_PI/2 : yaw+M_PI*1.5;
}



/*
 * pos： 前左上
 * Tr ： 前左上
*/
static void GetT_FromEuler_handler_static(Pose_txyzrpy pos, Eigen::Matrix4d &Tr) {

    Eigen::Affine3d affine;
    pcl::getTransformation(pos.x, pos.y, pos.z, pos.roll, pos.pitch, pos.yaw, affine);
    Tr = affine.matrix();

}

/*
 * Tr ： 前左上
 * pos： 前左上
*/
static void GetEuler_FromT_handler_static(Pose_txyzrpy &pos, Eigen::Matrix4d  Tr) {
    Eigen::Transform<double, 3, Eigen::Affine> affine (Tr);
    double x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles (affine, x, y, z, roll, pitch, yaw );

    pos.x = x;          pos.y = y;              pos.z = z;
    pos.pitch = pitch;      pos.roll = roll;   pos.yaw = yaw;
}



}
#endif // TRANSFORM_UTILS_HPP
