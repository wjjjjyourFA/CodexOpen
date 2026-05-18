

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


#include <pangolin/pangolin.h>



#include "define_utils.h"


enum SHOW_MODE{
    SHOW_INTENSITY,
    SHOW_Z,
}

#define show_mode SHOW_INTENSITY

class Visualizer{
public:
    Visualizer();
    


    void UpdateCurPoints_vec()
    {

        if( cur_points_num_ == 0 )
            return;

        if( frame_world_vec.size()>buffer_num_state )
            frame_world_vec.pop_front();

        PointCloudXYZI::Ptr frame_world_cp(new PointCloudXYZI(*frame_world));
        frame_world_vec.emplace_back(frame_world_cp);

        int points_num = 0, points_count = 0;
        for( int i=0; i<frame_world_vec.size(); i++ )
            points_num += frame_world_vec[i]->size();

        sum_points_num_ = points_num;

        std::cout << " sum points num : " << points_num << points_num  <<"  frame num : "<<frame_world_vec.size();

        if (target_vertexBuffer_ != nullptr)
            delete (target_vertexBuffer_);
        if (target_colorBuffer_ != nullptr)
            delete (target_colorBuffer_);

        pangolin::GlBuffer *vertexbuffer = new pangolin::GlBuffer(
                    pangolin::GlArrayBuffer, points_num, GL_FLOAT, 3, GL_STATIC_DRAW);
        pangolin::GlBuffer *colorbuffer = new pangolin::GlBuffer(
                    pangolin::GlArrayBuffer, points_num, GL_UNSIGNED_BYTE, 3, GL_STATIC_DRAW);



        float *dataUpdate = new float[points_num * 3];
        unsigned char *colorUpdate = new unsigned char[points_num * 3];
        uint8_t r, g, b;

        for( int i=0; i<frame_world_vec.size(); i++ ) {
            for( int ipt=0; ipt<frame_world_vec[i]->size(); ipt++ ) {
                dataUpdate[ipt * 3 + 0] = frame_world_vec[i]->points[ipt].x;
                dataUpdate[ipt * 3 + 1] = frame_world_vec[i]->points[ipt].y;
                dataUpdate[ipt * 3 + 2] = frame_world_vec[i]->points[ipt].z;

                switch(show_mode){
                    case SHOW_INTENSITY:
                        intensityToRGB( frame_world_vec[i]->points[ipt].intensity, b, g, r);
                    case SHOW_Z:
                        zToRGB( frame_world_vec[i]->points[ipt].z, r, g, b );
                    break;
                }
                    
                colorUpdate[ipt * 3 + 0] = r;
                colorUpdate[ipt * 3 + 1] = g;
                colorUpdate[ipt * 3 + 2] = b;
            }
        }


        (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * points_num,  0);
        (colorbuffer)->Upload(colorUpdate, sizeof(unsigned char) * 3 * points_num,  0);
        target_vertexBuffer_ = vertexbuffer;
        target_colorBuffer_ = colorbuffer;


        delete[] dataUpdate;
        delete[] colorUpdate;
    }



    void Run() {

        const int width = 1024, height = 768;
        pangolin::CreateWindowAndBind("manual slam", width, height);


        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);


    //    设定相机视角相关参数
        pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(width, height, 420, 420, width/2, height/2, 0.1, 1000),
                    // pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));
                    pangolin::ModelViewLookAt(0, 0, 50, 0, 0, 0, pangolin::AxisY));

        const int UI_SEG = 200;

    //    创建右侧 交互视图
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_SEG),1.0, -1.0 * width / height)
                .SetHandler(new pangolin::Handler3D(s_cam));

    //    创建左侧 控制面板
        pangolin::CreatePanel("ui")
                .SetBounds(0, 1.0, 0.0, pangolin::Attach::Pix(UI_SEG));

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        pangolin::OpenGlMatrix Twc; // camera to world
        Twc.SetIdentity();


        // pangolin::Var<bool> start_button            ("ui.Start Mapping", false, false);
        // pangolin::Var<bool> end_button              ("ui.End Mapping  ", false, false);
        // pangolin::Var<bool> save_button             ("ui.Save Map     ", false, false);
        pangolin::Var<int > point_size_button       ("ui.   Point Size", 2, 0, 8);
        pangolin::Var<int > buffer_num_button       ("ui.   Buffer num", 50, 50, 200);
        pangolin::Var<bool> show_points_button      ("ui.show 3DPoint ", true, true);
        pangolin::Var<bool> show_trajectory_button  ("ui.show trajectory", true, true);
        // pangolin::Var<bool> over_save_map_button    ("ui.   Over save map", false);
        pangolin::Var<double> fps_display_button    ("ui.   FPS     ", 0.0);
//        pangolin::Var<bool> settings_followCamera("ui.followCamera", false, true);
//        pangolin::Var<bool> reset_button            ("ui.Reset Map    ", false, false);
//        pangolin::Var<bool> clean_button            ("ui.Clean Map    ", false, false);


        // define variable

        std::thread t1 ;
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            s_cam.Follow(Twc);

            d_cam.Activate(s_cam);
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    //        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    //        std::cout<<"########### looping in main thread"<<std::endl;


//            if( pangolin::Pushed(reset_button) ) {
//               ResetStatus();
//            }

            // if( pangolin::Pushed(start_button) ) {
            //     t1 = std::thread(&OfflineViewer::StartMapping, this);
            // }

            // if( pangolin::Pushed(end_button) ) {
            //     EndMapping();
            // }

//            if( pangolin::Pushed(clean_button) ) {
//                CleanMap1();
//            }



            if( pangolin::Pushed(save_button) ) {
                std::cout<<"Saving map !!! "<<std::endl;
                if( SaveMap() )
                    over_save_map_button = true;
            }

            if (point_size_button.GuiChanged()) {
                point_size_state = point_size_button.Get();
                std::cout << "Point size changed to " << point_size_state << "\n";
            }

            if (buffer_num_button.GuiChanged()) {
                buffer_num_state = buffer_num_button.Get();
                std::cout << "Buffer num changed to " << buffer_num_state << "\n";
            }


            if( b_finish_mapping ) {
                b_finish_mapping = false;
                UpdateGLPoints();
                t1.join();
            }

            pangolin::glDrawAxis(3);  // 参数是坐标轴的大小


            {
                double tt0 = omp_get_wtime();

                std::lock_guard lock_update(mutex_update);
                double tt1 = omp_get_wtime();

                if( b_flag_update_points ) {

                    cur_points_num_ = laserCloud_world_cur->points.size();
                    LOG(INFO) << "cur map all points size : " << cur_points_num_ ;
                    UpdateCurPoints_vec();
                    fps_display_button = fps_display_state;
                    b_flag_update_points = false;
                    if( cur_points_num_>0 || old_points_num_>0)
                        b_display = true;

                    double tt2 = omp_get_wtime();
                    LOG(INFO) << " pangolin lock   : " << (tt1 - tt0)*1000 ;
                    LOG(INFO) << " pangolin update : " << (tt2 - tt1)*1000 ;
                }

            }



            if( b_display && show_points_button)
            {

                glDisable(GL_LIGHTING);                     // 禁用光照
                glPointSize(point_size_state);              // 设置渲染点时所用的点的大小

                target_colorBuffer_->Bind();                // 这通常意味着将一个对象（在本例中是一个颜色缓冲）绑定到当前的渲染上下文。
                glColorPointer(target_colorBuffer_->count_per_element,
                               target_colorBuffer_->datatype, 0, 0);    // 定义一个颜色数组
                glEnableClientState(GL_COLOR_ARRAY);        //启用颜色数组的使用


                target_vertexBuffer_->Bind();               // 将顶点缓冲区对象绑定到当前的渲染上下文
                glVertexPointer(target_vertexBuffer_->count_per_element,
                                target_vertexBuffer_->datatype, 0, 0);  //  定义一个顶点数组
                glEnableClientState(GL_VERTEX_ARRAY);       // 启用顶点数组的使用

                glDrawArrays(GL_POINTS, 0, sum_points_num_);    //  告诉OpenGL使用当前绑定并启用的顶点和颜色数组来绘制图形
                glDisableClientState(GL_VERTEX_ARRAY);      //在完成绘制后，禁用顶点数组
                target_vertexBuffer_->Unbind();             //解除了对顶点缓冲区的绑定
                glDisableClientState(GL_COLOR_ARRAY);       //关闭颜色数组的使用
                target_colorBuffer_->Unbind();              //解除对颜色缓冲区的绑定。


            }

            // trajectory
            if (show_trajectory_button)
            {
    //            if (settings_followCamera)
    //            {
    //                GetCurrentOpenGLCameraMatrix(Twc, Ow);
    //                Visualization3D_camera.Follow(Twc);
    //            }

                float colorRed[3] = {0, 1, 1};
                glColor3f(colorRed[0], colorRed[1], colorRed[2]);
                glLineWidth(10);
                glBegin(GL_LINE_STRIP);
                for (unsigned int i = 0; i < pose_vec.size(); i++) {
                    pcl::PointXYZ pose = pose_vec[i];
                    glVertex3d(pose.x, pose.y, pose.z);
                }
                glEnd();
            }

            pangolin::FinishFrame();
            usleep(100);


        }


    }
*/

    void Update(PointCloudXYZI::Ptr& frame_world_)
    {
        std::lock_guard lock_update(mutex_update);

        frame_world = frame_world_;
        b_update_frame = true;

    }


    PointCloudXYZI::Ptr frame_world;


    bool b_update_frame = false;

//    ui显示点云buffer
    pangolin::GlBuffer *target_vertexBuffer_;
    pangolin::GlBuffer *target_colorBuffer_;



    bool b_flag_exit;                   // 是否要退出mapping
    bool b_flag_update_points = false;  // 是否要更新ui中的点云

    std::mutex mutex_update;            // update point锁


    //    全局buffer
    std::deque<PointCloudXYZI::Ptr> frame_world_vec;

    int map_points_num_ = 0;
    int cur_points_num_ = 0;
    int sum_points_num_ = 0;
    int old_points_num_ = 0;
    PointCloudXYZI::Ptr laserCloud_world_map;   // ikdtree点云
    PointCloudXYZI::Ptr laserCloud_world_cur;   // cur点云
    PointCloudXYZI::Ptr laserCloud_world_all;   // ikdtree点云 + cur
    PointCloudXYZI::Ptr laserCloud_world_save;   // cur点云

    int count_save_pcd_num = 0;
    int thresh_save_pcd_num = -1;
    int save_cloud_idx = 0;


    std::vector<pcl::PointXYZ> pose_vec;

    std::vector<double> lid_time_vec;           // LIO 时间戳
    std::vector<SO3> lid_rot_vec;               // LIO rot
    std::vector<V3D> lid_pos_vec;               // LIO pos





    int lidar_type;
    int start_frame, stop_frame;                // 起始帧与结尾帧
    string ros_bag_path, config_file;           // bag文件，config文件
    string topic_lidar, topic_imu, topic_lidar_livox, topic_imu_livox, topic_globalPose;

    bool b_first_run = true;
    double fps_display_state = 0;
    int  point_size_state = 2;
    int  buffer_num_state = 50;

};