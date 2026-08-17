#include "modules/localization/fast_lio/include/imu_processing.hpp"

namespace fastlio {

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;                             // 初始化迭代次数
  Q = process_noise_cov();                       // 调用use-ikfom.hpp里面的process_noise_cov初始化噪声协方差
  cov_acc         = V3D(0.1, 0.1, 0.1);          // 加速度协方差初始化
  cov_gyr         = V3D(0.1, 0.1, 0.1);          // 角速度协方差初始化
  cov_bias_gyr    = V3D(0.0001, 0.0001, 0.0001); // 角速度bias协方差初始化
  cov_bias_acc    = V3D(0.0001, 0.0001, 0.0001); // 加速度bias协方差初始化
  mean_acc        = V3D(0, 0, -1.0);
  mean_gyr        = V3D(0, 0, 0);
  angvel_last     = Zero3d;                      // 上一帧角速度初始化
  Lidar_T_wrt_IMU = Zero3d;                      // lidar到IMU的位置外参初始化
  Lidar_R_wrt_IMU = Eye3d;                       // lidar到IMU的旋转外参初始化
  // last_imu_.reset(new ImuData());                // 上一帧imu初始化
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{ // 重置参数
  // 假设 IMU 是正着装的（Z朝上），作为初始猜测
  // 对于实际安装来说，需要 IMU 有一小段“静止数据”，用于初始化加速度计和陀螺仪的偏差
  mean_acc          = V3D(0, 0, -1.0);
  mean_gyr          = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;                // 是否需要初始化imu
  start_timestamp_  = -1;                  // 开始时间戳
  init_iter_num     = 1;                   // 初始化迭代次数
  v_imu_.clear();
  IMUpose.clear();                         // imu位姿清空
  // last_imu_.reset(new ImuData());          // 上一帧imu初始化
  cur_pcl_un_.reset(new PointCloudXYZI()); // 当前帧点云未去畸变初始化
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  // 在设置噪声强度参数，而不是 scale
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::set_whole_param(const V3D &transl, const M3D &rot, 
                                 const V3D &gyr, const V3D &acc, 
                                 const V3D &gyr_bias, const V3D &acc_bias)  
{ // 批量传入外部参数
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
  cov_gyr_scale = gyr;
  cov_acc_scale = acc;
  cov_bias_gyr = gyr_bias;
  cov_bias_acc = acc_bias;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
// void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N)
{ // IMU初始化：利用开始的IMU帧的平均值初始化状态量x
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  /* MeasureGroup 这个struct表示当前过程中正在处理的所有数据，
   * 包含 IMU队列 和 一帧lidar的点云，以及lidar帧的起始和结束时间
   * 初始化重力、陀螺仪偏差、acc和陀螺仪协方差，将加速度测量值归一化为单位重力   
   */
  V3D cur_acc, cur_gyr;
  
  // 如果为第一帧IMU
  if (b_first_frame_)
  { 
    Reset(); // 重置IMU参数
    N = 1;   // 将迭代次数置1
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front().linear_acceleration;  // IMU初始时刻的加速度
    const auto &gyr_acc = meas.imu.front().angular_velocity;     // IMU初始时刻的角速度
    // mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;                 // 第一帧加速度值作为初始化均值
    // mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;                 // 第一帧角速度值作为初始化均值
    mean_acc = imu_acc;
    mean_gyr = gyr_acc;
    first_lidar_time = meas.lidar_beg_time;                      // 将当前IMU帧对应的lidar起始时间 作为初始时间
  }

  // 如果IMU数据帧小于于 MAX_INI_COUNT 帧，则继续进行初始化
  // std::cout << "meas.imu.size(): " << meas.imu.size() << std::endl;

  // 根据所有IMU数据，计算平均值和方差
  // 前提是：必须在静止条件下，否则会被真实运动污染
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu.linear_acceleration;
    const auto &gyr_acc = imu.angular_velocity;
    // cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    // cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    cur_acc = imu_acc;
    cur_gyr = gyr_acc;
	
    // 根据当前帧和均值的差作为均值的更新
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // 有偏方差递推（MLE / biased）
    // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) / N;
    // cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) / N;
	
    // 无偏方差递推（对应 sample variance, unbiased）Fast-LIO
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout << "acc norm: " << cur_acc.norm() << " " << mean_acc.norm() << endl;

    N++;
  }

  state_ikfom init_state = kf_state.get_x();  // 在esekfom.hpp获得x_的状态
  // 得平均测量的单位方向向量 * 重力加速度预设值
  // 根据初始状态向量的模长估计重力，并归一化， 这是一个负值！ （此时车辆静止，故假设 |acc| = g ）
  // 用测到的加速度去“反推出重力在 IMU 坐标系中的表达”
  // G_m_s2 = 9.81 ==> GRAV = 9.81 * (0, 0, -1)

  // way 1 mean_acc / mean_acc.norm() ==> (0.aa, 0.bb, -0.cc) ==> 适应非水平安装的IMU
  // 实际地面平台实验结果：将 IMU 和 lidar 数据，手动调整到水平坐标系传入，建图结果才正确
  // init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  init_state.grav = S2(- (mean_acc / mean_acc.norm()) * G_m_s2);
  // way 2 利用静止状态测量 rot，或传入标定的 rot；将 grav 放在“世界系期望值”
  // !! 实际地面平台实验结果：这里指定不是 世界坐标系 的重力，是 IMU 坐标系的重力
  // 详见：==> save the poses at each IMU measurements
  // init_state.grav = S2(Eigen::Vector3d(0, 0, -G_m_s2));
  
  // 非定位状态时，需要初始化 rot；定位状态，使用定位提供的 rot；
  if (init_mode == 0) {
    /* way 1
    init_state.rot = Eye3d;
    init_state.rot = Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    */
    /* way 2 利用 标定的 rot
    Eigen::Matrix4d imu_ext = Eigen::Matrix4d::Identity();
    imu_ext << 1.0, 0.0, 0.0, 0.0, 
              0.0, 0.6428, -0.7660, 0.0, 
              0.0, 0.7660, 0.6428, 0.0, 
              0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d R_ext = imu_ext.block<3,3>(0,0);
    // std::cout << std::fixed << std::setprecision(6);
    // std::cout << "R_ext.matrix(): \n" << R_ext << std::endl;
    init_state.rot = SO3(R_ext);
    // std::cout << "init_state.rot.matrix(): \n" << init_state.rot.toRotationMatrix() << std::endl;
    */
    // /* way 3 利用 静止状态测量 rot
    Eigen::Vector3d z_imu = -mean_acc.normalized();
    Eigen::Vector3d z_world(0, 0, -1);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_imu, z_world);
    // std::cout << "q.toRotationMatrix(): \n" << q.toRotationMatrix() << std::endl;
    init_state.rot = SO3(q.w(), q.x(), q.y(), q.z());
    // std::cout << "init_state.rot.matrix(): \n" << init_state.rot.toRotationMatrix() << std::endl;
    // */
  }

  init_state.bg  = mean_gyr;                  // 角速度测量均值作为陀螺仪偏差
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;  // 将lidar和imu外参传入
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);              // 将初始化后的状态传入esekfom.hpp中的x_

  // 在esekfom.hpp获得P_的协方差矩阵
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;        // 外参 R
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;    // 外参 t
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;   // bias a
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;    // bias g
  init_P(21,21) = init_P(22,22) = 0.00001;                  // 重力？ S2类型
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
// void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_out)
{ // !! 严格保证，传入的数据是: imu_start_time < lidar_start_time < lidar_end_time < imu_end_time
  // 反向传播
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  /*** 将上一帧最后尾部的imu 添加到 当前帧头部的imu ***/
  auto v_imu = meas.imu;        // 取出当前帧的IMU队列
  // 将上一帧最后尾部的imu添加到当前帧头部的imu
  // 新的数据结构中，自带上一帧lidar末尾的IMU数据，因此这里不需要再添加
  // v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front().timestamp;
  // 拿到当前帧尾部的imu的时间
  const double &imu_end_time = v_imu.back().timestamp;
  // 点云开始和结束的时间戳
  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;
 
  /* debug
  std::cout << std::fixed << std::setprecision(15) << std::endl;
  std::cout << "imu_beg_time: " << imu_beg_time << std::endl;
  std::cout << "imu_beg_second_time: " << v_imu[1].timestamp << std::endl;
  std::cout << "imu_beg_third_time: " << v_imu[2].timestamp << std::endl;
  std::cout << "imu_end_time: " << imu_end_time << std::endl;
  std::cout << "pcl_end_time: " << pcl_end_time << std::endl;
  // abort();
  */

  if (lidar_type == MARSIM) {
    pcl_beg_time = last_lidar_end_time_;
    pcl_end_time = meas.lidar_beg_time;
  }

  /*** sort point clouds by offset time ***/
  // 根据点云中每个点的时间戳对点云进行重排序
  pcl_out = *(meas.lidar);
  // 这里 "intensity"字段 中存放时间戳（在preprocess.cpp中）
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  // 获取上一次KF估计的后验状态作为本次IMU预测的初始状态
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  // 将初始状态加入IMUpose中,包含有时间间隔，上一帧加速度，上一帧角速度，上一帧速度，上一帧位置，上一帧旋转矩阵
  // 以 lidar_beg_time 为基准，计算这一帧的 IMUpose vector
  // offset_time = 0 对应 lidar_beg_time
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  /*** 前向传播 ***/
  // angvel_avr为平均角速度，acc_avr为平均加速度，acc_imu为imu加速度，vel_imu为imu速度，pos_imu为imu位置
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  // IMU旋转矩阵 消除运动失真的时候用
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  // 遍历本次估计的所有IMU测量并且进行积分，离散中值法 前向传播
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);      // 拿到当前帧的imu数据
    auto &&tail = *(it_imu + 1);  // 拿到下一帧的imu数据
    // 判断时间先后顺序：下一帧时间戳是否小于上一帧结束时间戳 不符合直接continue
    // first imu ==> imu_head_time_1 < lidar_start_time < imu_tail_time_2
    if (tail.timestamp < last_lidar_end_time_) continue;
    // std::cout << "tail.timestamp: " << tail.timestamp << std::endl;

    // !! 严格意义上，这里应该 截断积分
    // 中值积分
    // angvel_avr<<0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
    //             0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
    //             0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
    // acc_avr   <<0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
    //             0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
    //             0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);
    angvel_avr = 0.5 * (head.angular_velocity + tail.angular_velocity);
    acc_avr = 0.5 * (head.linear_acceleration + tail.linear_acceleration);

    // fout_imu << setw(10) << head.header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;
    
    // 恢复IMU数据的模长是 |acc| ≈ 1.0 ：尺度校正（scale correction）
    // 除以初始化的IMU模长，再通过重力数值 * 9.8 ，将加速度从 g 转换为 m/s^2
    acc_avr = acc_avr * (G_m_s2 / mean_acc.norm()); // - state_inout.ba;

    // 如果IMU开始时刻早于上次雷达最晚时刻(因为将上次最后一个IMU插入到此次开头了，所以会出现一次这种情况)
    // 两个imu时间分别在 ldiar_beg_time 一前一后
    if(head.timestamp < last_lidar_end_time_)
    {
      // 从上次雷达时刻末尾开始传播 计算与此次IMU结尾之间的时间差
      // 这里决定了 每帧雷达 都严格变换到 lidar_end_time 时刻
      dt = tail.timestamp - last_lidar_end_time_;
      // dt = tail.header.stamp.toSec() - pcl_beg_time;
    }
    else if (tail.timestamp > pcl_end_time)
    {
      // 最后一帧的imu数据的对齐，放在循环外手动做，防止预测过头
      // last_imu ==> imu_head_time_k-1 < lidar_end_time < imu_tail_time_k
      // imu_end_time_k-1 ==> 最左逼近 pcl_end_time 的 imu_end_time
      dt = pcl_end_time - head.timestamp;
    } 
    else {
      // 两个IMU时刻之间的时间间隔
      dt = tail.timestamp - head.timestamp;
      // dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    // 两帧IMU的中值作为输入in  用于前向传播
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    // 配置观测的协方差矩阵
    // TODO：可以根据IMU的精度动态调整协方差矩阵
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    // 输入 时间，观测协方差，观测数据，更新kf.x_， kf.P_
    kf_state.predict(dt, Q, in);  // IMU前向传播，每次传播的时间间隔为dt

    /* save the poses at each IMU measurements */
    // 更新IMU状态为积分后的状态
    imu_state = kf_state.get_x();
    // /* way 1
    // 更新上一帧角速度 = 平均帧角速度-bias  
    angvel_last = angvel_avr - imu_state.bg;
    // 更新上一帧世界坐标系下的加速度 = R*(加速度-bias) - g
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
    // 各个轴的加速度观察值，都要减去重力的影响
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    // */
    /* way 2
    // 更新上一帧角速度 = 后一帧角速度-bias  
    angvel_last = V3D(tail.angular_velocity[0], tail.angular_velocity[1], tail.angular_velocity[2]) - imu_state.bg;
    // 更新上一帧世界坐标系下的加速度 = 使用后一帧线速度
    // 只有在“完全静止”时，才成立：  ==> * G_m_s2 / mean_acc.norm();  ==> 将加速度从 g 转换为 m/s^2
    acc_s_last = V3D(tail.linear_acceleration[0], tail.linear_acceleration[1], tail.linear_acceleration[2]) * G_m_s2 / mean_acc.norm(); 
    acc_s_last = imu_state.rot * (acc_s_last - imu_state.ba) + imu_state.grav;
    */
    // 后一个IMU时刻距离此次雷达开始的时间间隔
    double &&offs_t = tail.timestamp - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  // 把最后一帧IMU测量也补上
  // !! 其实这里应该严格意义是 pcl_end_time - imu_end_time_k-1 ( imu_end_time_k < pcl_end_time < imu_end_time)
  /* way 1 用“粗积分 + EKF吸收误差”
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  // dt = abs(pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  */
  // way 2 已经严格积分到 lidar_end_time
  // 循环结束后“不再补 dt hack”，直接认为已经对齐
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();          // 保存最后一个IMU测量，以便于下一帧使用
  last_lidar_end_time_ = pcl_end_time;  // 保存这一帧最后一个雷达测量的结束时间，以便于下一帧使用

  /*** undistort each lidar point (backward propagation) ***/
  /*** 消除每个激光雷达点的失真（反向传播）***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;

  if(lidar_type != MARSIM){
    // it_pcl 是共享状态，由外部维护，只会推进一次
    auto it_pcl = pcl_out.points.end() - 1;
    // 遍历每个IMU帧，倒序
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu<<MAT_FROM_ARRAY(head->rot);       // 拿到前一帧的IMU旋转矩阵
      // cout << "head imu acc: " << acc_imu.transpose() << endl;
      vel_imu<<VEC_FROM_ARRAY(head->vel);     // 拿到前一帧的IMU速度
      pos_imu<<VEC_FROM_ARRAY(head->pos);     // 拿到前一帧的IMU位置
      acc_imu<<VEC_FROM_ARRAY(tail->acc);     // 拿到后一帧的IMU加速度
      angvel_avr<<VEC_FROM_ARRAY(tail->gyr);  // 拿到后一帧的IMU角速度

      // 之前点云按照时间从小到大排序过，IMUpose也同样是按照时间从小到大push进入的
      // 此时从IMUpose的末尾开始循环，也就是从时间最大处开始，因此只需要判断 点云时间 > IMU head 时刻 即可   
      // 不需要判断 点云时间 < IMU tail
      // for(; it_pcl->intensity / double(1000) > head->offset_time; it_pcl --)
      for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
      {
        // 点到IMU开始时刻的时间间隔 
        // dt = it_pcl->intensity / double(1000) - head->offset_time;
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame, using only the rotation
          * Note: Compensation direction is INVERSE of Frame's moving direction
          * So if we want to compensate a point at timestamp-i to the frame-e
          * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
        // 点it_pcl所在时刻的旋转：前一帧的IMU旋转矩阵 * exp(后一帧角速度*dt)   
        M3D R_i(R_imu * Exp(angvel_avr, dt));

        // 点所在时刻的位置(雷达坐标系下)
        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        // 从点所在的世界位置-雷达末尾世界位置
        // 1. LiDAR → IMU（时刻 i）
        // P_imu_i = offset_R_L_I * P_i + offset_T_L_I
        // 2. IMU(i) → 世界系 ==> T_ei = ti - te
        // P_world = R_i * P_imu_i + T_ei
        // 3. 世界系 → 当前 IMU（时刻 e） ==> 从这里推导出， imu_state.rot 代表的是 IMU 到世界坐标系的旋转关系
        // P_imu_e = imu_state.rot.conjugate() * P_world
        // 4. IMU → LiDAR（当前时刻）
        // P_lidar_e = offset_R_L_I.conjugate() * (P_imu_e - offset_T_L_I)
        V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
     
        // save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin()) break;
      }
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
// void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  // double t1,t2,t3;
  // t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    // 开头几帧，需要初始化IMU参数
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    // IMU_init 中已经赋值，这里不需要再赋值
    // last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      // IMU_init() 中已经赋值 cov_acc、cov_gyr ，这里为什么还要重新赋值？
      // IMU_init() 是为了初始化 KF，而实际预测中，用重设的固定值 cov_acc、cov_gyr 更稳定

      // 动态调整噪声协方差（cov）：根据当前状态，动态改变滤波器对 IMU 的信任程度
      // 自适应 scale ==> m/s^2
      // 通常只对 acc 做自适应，gyr 很少需要。

      /* way 1 根据重力模长误差 → 推导噪声应该变多少; 将加速度从 g 转换为 m/s^2
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      */
      /* way 2 基于“初始化质量”的人为调节
      double ratio = fabs(mean_acc.norm() - G_m_s2);
      double scale = 1.0 + k * ratio;
      cov_acc = cov_acc_scale * scale;
      */
      // way 3 用固定调参值设置噪声大小
      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;

      imu_need_init_ = false;

      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      std::cout << "IMU Initial Done" << std::endl;
      // fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  // t2 = omp_get_wtime();
  // t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}

}
