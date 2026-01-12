#include <iostream>
#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/register_point_struct.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <iostream>
#include <fstream>
#include "colors.hpp"
#include <octomap/ColorOcTree.h>
#include <iostream>
#include <stdio.h>

#include <sys/time.h>

#include <unistd.h>
#include <random>

using namespace std;
using namespace octomap;

#define multi_map 0

#define Plot_all 0

#define Piece_num_x 20    //20 9 dynamic vehicles heigth_bias -1.0, z_thresh 0.5

#define Target_piece_x 9   //start from 0 down

#define Piece_num_y 8    //20 9 dynamic vehicles heigth_bias -1.0, z_thresh 0.5

#define Target_piece_y 4   //start from 0 down

#define Sort_by_x 1

#define Sort_by_y 1

#define height_bias 0.5   //meters

#define downsample 1

#define z_thresh -100   //default: -100 meters

struct pointXYZ
{
    float x;
    float y;
    float z;
};

double tic() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


void toc(double t) {
    double s = tic();
    std::cout<<std::max(0., (s-t)*1000)<<" ms"<<std::endl;
}

float x_min = 10000000000;
float x_max = -10000000000;
float y_min = 10000000000;
float y_max = -10000000000;
float z_min = 10000000000;
float z_max = -10000000000;

bool plot_xy(float input_x, float input_y, bool sort_by_x, bool sort_by_y, int piece_num_all_x, int num_target_x, int piece_num_all_y, int num_target_y);

int main()
{

    // Define random generator with Gaussian distribution
    double mean = 0.0;//均值
    double stddev = 0.18;//标准差  0  0.18   0.3

    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    double t = tic();
    //0526-good-filter
    string m_filename1 = "./octomap/loam_edge_map.ot";
    AbstractOcTree* tree_pnt1 = AbstractOcTree::read(m_filename1);
    const ColorOcTree& tree1 = ((const ColorOcTree&) *tree_pnt1);
    //    string m_filename2 = "./octomap/extra/loam_edge_map.ot";    //edge plane
    //    AbstractOcTree* tree_pnt2 = AbstractOcTree::read(m_filename2);
    //    const ColorOcTree& tree2 = ((const ColorOcTree&) *tree_pnt2);

    //读取地图位置
    //    int Offset1[3]; // x y z roll pitch tree_max_z tree_min_z
    //    char filename1[100] = "./octomap/mapper_offset.txt";
    //    FILE* fp = fopen(filename1, "r");
    //    for(int i=0; i<3; i++)
    //        fscanf(fp, "%d", &Offset1[i]);
    //    fclose(fp);

    //    int Offset2[3]; // x y z roll pitch tree_max_z tree_min_z
    //    char filename2[100] = "./octomap/extra/mapper_offset.txt";
    //    FILE* fp2 = fopen(filename2, "r");
    //    for(int i=0; i<3; i++)
    //        fscanf(fp2, "%d", &Offset2[i]);
    //    fclose(fp2);

    //    int delta[3];
    //    for(int i=0; i<3; i++)
    //        delta[i] = Offset2[i] - Offset1[i];

    //    double bias[3];
    //    for(int i=0;i<3;i++)
    //        bias[i] = delta[i]*1.0*0.01;

    //pcl ,for show3DMap
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr showClouds(new pcl::PointCloud<pcl::PointXYZRGB>);
    //    for (int i = 0; i < treeNum; i++)
    //    {
    //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr showCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //        showClouds[i] = showCloud;
    //    }



    //create window "show"
    pcl::PointXYZRGB o;
    pcl::visualization::PCLVisualizer *pp = NULL;
    if (pp == NULL)
    {

        //add by rrk 20190327 3D visualization
        float azimuth_local = M_PI;    //rad
        float view_dis = 200;   //200
        float view_x = view_dis * sin(azimuth_local);
        float view_y = view_dis;
        float view_z = -view_dis * cos(azimuth_local);

        pp = new pcl::visualization::PCLVisualizer ("map1!");
        pp->addCoordinateSystem(1.0);
        pp->removeAllPointClouds();
        pp->removeAllShapes();
        pp->setBackgroundColor(0.0,0.0,0.0);
        //pp->setCameraPosition(view_x, view_y, view_z, 0, 0, 0, 0);
    }

    //exact point

    point3d temp_point;
    showClouds->points.clear();

    //1
    int index1 = 0;
    int index2 = 0;


    //add by rrk 20200226 break into pieces

    for(ColorOcTree::tree_iterator it = tree1.begin_tree(16),end=tree1.end_tree(); it!= end; ++it)
    {
        if (it.isLeaf())  //voxels for leaf nodes
            if (tree1.isNodeOccupied(*it))  // occupied voxels
            {
                //          if (tree.isNodeAtThreshold(*it)) {
                temp_point = it.getCoordinate();

                o.x=temp_point(0);
                o.y=temp_point(1);
                o.z=temp_point(2);

                if(o.x<x_min)
                    x_min = o.x;
                if(o.x>x_max)
                    x_max = o.x;
                if(o.y<y_min)
                    y_min = o.y;
                if(o.y>y_max)
                    y_max = o.y;
                if(o.z<z_min)
                    z_min = o.z;
                if(o.z>z_max)
                    z_max = o.z;
            }
    }

    cout<<"x: "<<x_min<<" "<<x_max<<endl;
    cout<<"y: "<<y_min<<" "<<y_max<<endl;
    cout<<"z: "<<z_min<<" "<<z_max<<endl;

    //calculate targets and z_mean
    vector<pointXYZ> targets;
    float z_mean = 0;
    for(ColorOcTree::tree_iterator it = tree1.begin_tree(16),end=tree1.end_tree(); it!= end; ++it)
    {
        if (it.isLeaf())  //voxels for leaf nodes
            if (tree1.isNodeOccupied(*it))  // occupied voxels
            {
                //          if (tree.isNodeAtThreshold(*it)) {
                temp_point = it.getCoordinate();

                o.x=temp_point(0);
                o.y=temp_point(1);
                o.z=temp_point(2);

                bool inside_target = plot_xy(o.x,o.y,Sort_by_x,Sort_by_y,Piece_num_x,
                                             Target_piece_x,Piece_num_y,Target_piece_y);

                if(inside_target)
                {
                    pointXYZ tmp;
                    tmp.x = o.x;
                    tmp.y = o.y;
                    tmp.z = o.z;
                    targets.push_back(tmp);
                    z_mean += o.z;
                }

            }
    }
    z_mean = z_mean/targets.size();

    cout<<"mean_z: "<<z_mean<<endl;

    if(Plot_all)
    {
        for(ColorOcTree::tree_iterator it = tree1.begin_tree(16),end=tree1.end_tree(); it!= end; ++it)
        {
            if (it.isLeaf())  //voxels for leaf nodes
                if (tree1.isNodeOccupied(*it))  // occupied voxels
                {
                    //          if (tree.isNodeAtThreshold(*it)) {
                    temp_point = it.getCoordinate();

                    o.x=temp_point(0);
                    o.y=temp_point(1);
                    o.z=temp_point(2);

                    if(stddev>0)
                    {
                        o.x += 1.0*dist(generator);
                        o.y += 1.0*dist(generator);
                        o.z += 1.0*dist(generator);
                    }


                    int height = o.z;

                    height = (o.z+1)*10;
                    if(height<0) height = 0;
                    if(height>49) height = 49;
                    o.r = jet_color_map[height*10][0];
                    o.g = jet_color_map[height*10][1];
                    o.b = jet_color_map[height*10][2];

                    index1++;
                    if(index1%downsample == 0 && o.z > z_thresh)
                        showClouds->points.push_back(o);
                }
        }
    }
    else
    {
        for(int i=0;i<targets.size();i++)
        {
            o.x=targets[i].x;
            o.y=targets[i].y;
            o.z=targets[i].z;

            if(o.z > 2)
                continue;

            if(o.y > 100 && o.z > 1)
                continue;

//            if(o.x<-30)
//                continue;

            if(o.y > 100 && o.z > 0.2 && o.x<-30)
                continue;

            if(stddev>0)
            {
                o.x += 1.0*dist(generator);
                o.y += 1.0*dist(generator);
                o.z += 1.0*dist(generator);
            }

            int height;
            height = (o.z - z_mean + height_bias +1)*10;
            if(height<0) height = 0;
            if(height>49) height = 49;
            o.r = jet_color_map[height*10][0];
            o.g = jet_color_map[height*10][1];
            o.b = jet_color_map[height*10][2];

            index1++;
            if(index1%downsample == 0 && o.z > z_thresh)
                showClouds->points.push_back(o);
        }
    }

    //2
#if multi_map
    for(ColorOcTree::tree_iterator it = tree2.begin_tree(16),end=tree2.end_tree(); it!= end; ++it)
    {
        if (it.isLeaf())  //voxels for leaf nodes
            if (tree2.isNodeOccupied(*it))  // occupied voxels
            {
                //          if (tree.isNodeAtThreshold(*it)) {
                temp_point = it.getCoordinate();

                o.x=temp_point(0) + bias[0];
                o.y=temp_point(1) + bias[1];
                o.z=temp_point(2) + bias[2];


                int height = o.z;

                height = (o.z+1)*10;
                if(height<0) height = 0;
                if(height>49) height = 49;
                o.r = jet_color_map[height*10][0];
                o.g = jet_color_map[height*10][1];
                o.b = jet_color_map[height*10][2];

                index2++;
                if(index2%5 == 0)
                    showClouds->points.push_back(o);
            }
    }
#endif

    toc(t);


    //show showClouds;

    pp->addPointCloud(showClouds);



    pp->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);

    pp->spin();
    //    pp->spinOnce(10);

    delete pp;
}
//-550.65 1101.3
bool plot_xy(float input_x, float input_y, bool sort_by_x, bool sort_by_y,
             int piece_num_all_x, int num_target_x,int piece_num_all_y, int num_target_y)
{
    if(sort_by_x && !sort_by_y)
    {
        float x_start = x_min + (num_target_x)*(x_max - x_min)/piece_num_all_x;
        float x_end = x_min + (num_target_x+1)*(x_max - x_min)/piece_num_all_x;
        if(input_x>=x_start && input_x<=x_end)
            return true;
        else
            return false;
    }
    if(!sort_by_x && sort_by_y)
    {
        float y_start = y_min + (num_target_y)*(y_max - y_min)/piece_num_all_y;
        float y_end = y_min + (num_target_y+1)*(y_max - y_min)/piece_num_all_y;
        if(input_y>=y_start && input_y<=y_end)
            return true;
        else
            return false;
    }
    if(sort_by_x && sort_by_y)
    {
        float x_start = x_min + (num_target_x)*(x_max - x_min)/piece_num_all_x;
        float x_end = x_min + (num_target_x+1)*(x_max - x_min)/piece_num_all_x;
        float y_start = y_min + (num_target_y)*(y_max - y_min)/piece_num_all_y;
        float y_end = y_min + (num_target_y+1)*(y_max - y_min)/piece_num_all_y;
        if(input_y>=y_start && input_y<=y_end && input_x>=x_start && input_x<=x_end)
            return true;
        else
            return false;
    }
}

