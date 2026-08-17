

#include "ImuProcess.h"



ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
    init_iter_num = 1;
    Q = process_noise_cov();
    cov_acc       = V3D(0.1, 0.1, 0.1);
    cov_gyr       = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
    mean_acc      = V3D(0, 0, -1.0);
    mean_gyr      = V3D(0, 0, 0);
    angvel_last     = Zero3d;
    Lidar_T_wrt_IMU = Zero3d;
    Lidar_R_wrt_IMU = Eye3d;
    last_imu_ = std::make_shared<ImuData>();
    extrinT.assign(3, 0.0);
    extrinR.assign(9, 0.0);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
    mean_acc      = V3D(0, 0, -1.0);
    mean_gyr      = V3D(0, 0, 0);
    angvel_last       = Zero3d;
    imu_need_init_    = true;
    start_timestamp_  = -1;
    init_iter_num     = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_ = std::make_shared<ImuData>();
    cur_pcl_un_.reset(new PointCloudXYZI());
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

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

    V3D cur_acc, cur_gyr;

    if (b_first_frame_) //如果为第一帧IMU
    {
        Reset();    //重置IMU参数
        N = 1;      //将迭代次数置1
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;        //第一帧加速度值作为初始化均值
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;        //第一帧角速度值作为初始化均值
        first_lidar_time = meas.lidar_bag_time;             //将当前IMU帧对应的lidar起始时间 作为初始时间
    }

//    读取当前帧所有的imu观测信息，计算acc，gyr的均值，协方差
    for (const auto &imu : meas.imu)
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc      += (cur_acc - mean_acc) / N;
        mean_gyr      += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        N ++;
    }


//    修改初始估计的重力分量，陀螺仪bias-bg，传入旋转和平移的外参
    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);    //    根据初始状态向量的模长估计重力，并归一化， 这是一个负值！ （此时车辆静止，故假设 |acc| = g ）
    init_state.bg  = mean_gyr;  // 初始化陀螺仪bias为角速度均值
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    kf_state.change_x(init_state);


    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;          // 外参 R
    init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;      // 外参 t
    init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;     // bias a
    init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;      // bias g
    init_P(21,21) = init_P(22,22) = 0.00001;                    // 重力？ S2类型
    kf_state.change_P(init_P);
    last_imu_ = meas.imu.back();

//    pos:(0, 0, 0)  bg:(-0.00a, -0.000b, 0.000c),  ba:(0, 0, 0)   grav:( -0.03013 0.719264 -9.78255)
    std::cout << "IMU init new -- init_state  " << init_state.pos  <<" " << init_state.bg <<" " << init_state.ba <<" " << init_state.grav << std::endl;

}

// 后向传播，点云去畸变
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;  // imu数据序列
    v_imu.push_front(last_imu_);    // 插入上一帧的最后一个imu数据
    const double &imu_beg_time = v_imu.front()->timestamp;   // imu序列起始时间
    const double &imu_end_time = v_imu.back()->timestamp;    // imu序列结束时间
    const double &pcl_beg_time = meas.lidar_bag_time;       // 点云起始时间
    const double &pcl_end_time = meas.lidar_end_time;       // 点云结束时间

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);  // 点云按照时间排序
    // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", "
    //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose.clear();
    // 设定初始时刻相对状态（
    // 相对时间，加速度，角速度，速度，位置，旋转矩阵）
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    double dt = 0;

    input_ikfom in; // 输入的观测量，包含acc, gyr
//    前向传播，对每一帧imu进行预测，预测传入参数包括 (in, dt, Q)
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->timestamp < last_lidar_end_time_)    continue;

        angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                    0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                    0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                    0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                    0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);


        acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

        if(head->timestamp < last_lidar_end_time_)  { // 两个imu时间分别在 ldiar_beg_time 一前一后
            dt = tail->timestamp - last_lidar_end_time_;     // 这里是上一帧lidar结束时间
        }
        else     {    // 两个imu时间都在 lidar_beg_time 后面
            dt = tail->timestamp - head->timestamp;
        }

//        观测数据， 观测的协方差矩阵
        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

//        输入 时间，观测协方差，观测数据，更新kf.x_， kf.P_
        kf_state.predict(dt, Q, in);
        imu_state = kf_state.get_x();

//        减掉估计的随机游走噪声
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);

//        减掉估计的重力
        for(int i=0; i<3; i++)
        {
            acc_s_last[i] += imu_state.grav[i];
        }

        double &&offs_t = tail->timestamp - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    }

//    预测帧尾状态
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0; // 一般都是1，在syncMesure中确保了 pcl_end_time 总是大于 imu_end_time
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q, in);

    imu_state = kf_state.get_x();
    last_imu_ = meas.imu.back();                //保存最后一个IMU测量，以便于下一帧使用
    last_lidar_end_time_ = pcl_end_time;        //保存这一帧最后一个雷达测量的结束时间，以便于下一帧使用

//    对每个点进行去畸变
    if (pcl_out.points.begin() == pcl_out.points.end()) return;
    auto it_pcl = pcl_out.points.end() - 1;


//    IMUpose： time acc gyr vel pos rot
//    遍历每个IMU帧，倒序
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu<<MAT_FROM_ARRAY(head->rot);
        vel_imu<<VEC_FROM_ARRAY(head->vel);
        pos_imu<<VEC_FROM_ARRAY(head->pos);
        acc_imu<<VEC_FROM_ARRAY(tail->acc);
        angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

//        倒序遍历所有点云，每一个for处理一段 (head, tail) 之间的点云数据
        for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
        {
            dt = it_pcl->curvature / double(1000) - head->offset_time;


//            compensate a point at timestamp-i to the frame end
//            P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei)
            M3D R_i(R_imu * Exp(angvel_avr, dt));   // 点it_pcl所在时刻的旋转 = head时刻的旋转 * exp(平均角速度*dt)
            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);     //  点所在的世界位置 - end世界位置
            V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin()) break;
        }
    }
}

bool ImuProcess::Process(
    const MeasureGroup &meas,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
    PointCloudXYZI::Ptr cur_pcl_un_)
{
    double t1,t2;
    t1 = omp_get_wtime();

    if(meas.imu.empty()) {return false;};
    if (meas.lidar == nullptr) {
        throw std::invalid_argument("IMU process requires a lidar cloud");
    }

    // 首先进行imu初始化，初始化完成前不进行去畸变操作
    if (imu_need_init_)
    {
        /// The very first lidar frame
        IMU_init(meas, kf_state, init_iter_num);

        imu_need_init_ = true;

        last_imu_   = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num > MAX_INI_COUNT)
        {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            LOG(INFO) << "IMU Initial Done";
            fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
        }

        return false;
    }

    UndistortPcl(meas, kf_state, *cur_pcl_un_);

    t2 = omp_get_wtime();

    LOG(INFO) << "imu process cost time : " << (t2-t1)*1000 ;
    return true;
}
