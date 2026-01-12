#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <random>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/register_point_struct.h>
#include "pcl/visualization/pcl_visualizer.h"

#include "colors.hpp"

using namespace std;
using namespace octomap;

#define multi_map 0
#define Plot_all 1

#define Piece_num_x 3    //20 9 dynamic vehicles heigth_bias -1.0, z_thresh 0.5
#define Target_piece_x 2   //start from 0 down
#define Piece_num_y 3    //20 9 dynamic vehicles heigth_bias -1.0, z_thresh 0.5
#define Target_piece_y 2  //start from 0 down

#define Sort_by_x 1
#define Sort_by_y 1
#define z_thresh -1000   //default: -100 meters
#define height_bias 2.5   //meters
#define SUB_MAP_X_SIZE 100
#define SUB_MAP_Y_SIZE 100
#define downsample 3

int sub_map_rows = 0, sub_map_cols = 0, sub_map_num = 0;

float x_min = 10000000000;
float x_max = -10000000000;
float y_min = 10000000000;
float y_max = -10000000000;
float z_min = 10000000000;
float z_max = -10000000000;

typedef pcl::PointXYZ pointXYZ;
typedef pcl::PointXYZRGB pointXYZRGB;

string m_filename1 = "/media/jojo/Study/2021_wj/src/tools_rrk/res_20220524/school_map/point_cloud_map.ot";

double tic() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

void toc(double t) {
    double s = tic();
    std::cout<<std::max(0., (s-t)*1000)<<" ms"<<std::endl;
}

void CreatSubMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

void ShowMapCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

bool plot_xy(float input_x, float input_y, bool sort_by_x, bool sort_by_y, int piece_num_all_x, int num_target_x, int piece_num_all_y, int num_target_y);

int main()
{
    double t = tic();

//    string m_filename1 = "./octomap/submap_hexi/142.ot";
//    string m_filename2 = "./octomap/submap_hexi/143.ot";
//    string m_filename1 = "./octomap/submap_hexi_down/1.ot";
//    string m_filename2 = "./octomap/submap_hexi_down/2.ot";
    AbstractOcTree* tree_pnt1 = AbstractOcTree::read(m_filename1);
    const ColorOcTree& tree1 = ((const ColorOcTree&) *tree_pnt1);
//    AbstractOcTree* tree_pnt2 = AbstractOcTree::read(m_filename2);
//    const ColorOcTree& tree2 = ((const ColorOcTree&) *tree_pnt2);

//    ColorOcTree tree1 ;
//    ColorOcTree tree2 ;
//    tree1.read(m_filename1);
//    tree2.read(m_filename2);
//        std::cout<<tree1.size()<<std::endl;
//        points_num == 104613336;   约500帧点云  联东U谷
//        points_num == 47651364;    约200帧点云  河西Edge
//        points_num == 32094144;    约150帧点云  河西Plane

    toc(t);

    // Define random generator with Gaussian distribution
    double mean = 0.0;//均值
    double stddev = 0.0;//标准差  0  0.18   0.3

    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB o;

    point3d temp_point;
    int point_index = 0;

    toc(t);

    for(ColorOcTree::tree_iterator it = tree1.begin_tree(16),end=tree1.end_tree(); it!= end; ++it)
    {
        if (it.isLeaf())  //voxels for leaf nodes
            if (tree1.isNodeOccupied(*it))  // occupied voxels
            {
                //          if (tree.isNodeAtThreshold(*it)) {
                temp_point = it.getCoordinate();

                x_min = std::min(temp_point(0), x_min);
                y_min = std::min(temp_point(1), y_min);
                z_min = std::min(temp_point(2), z_min);

                x_max = std::max(temp_point(0), x_max);
                y_max = std::max(temp_point(1), y_max);
                z_max = std::max(temp_point(2), z_max);

            }
    }
//    for(ColorOcTree::tree_iterator it = tree2.begin_tree(16),end=tree1.end_tree(); it!= end; ++it)
//    {
//        if (it.isLeaf())  //voxels for leaf nodes
//            if (tree1.isNodeOccupied(*it))  // occupied voxels
//            {
//                //          if (tree.isNodeAtThreshold(*it)) {
//                temp_point = it.getCoordinate();

//                x_min = std::min(temp_point(0), x_min);
//                y_min = std::min(temp_point(1), y_min);
//                z_min = std::min(temp_point(2), z_min);

//                x_max = std::max(temp_point(0), x_max);
//                y_max = std::max(temp_point(1), y_max);
//                z_max = std::max(temp_point(2), z_max);

//            }
//    }
    sub_map_cols = (x_max - x_min)/100 + 1;
    sub_map_rows = (y_max - y_min)/100 + 1;
    sub_map_num = sub_map_cols * sub_map_rows;
    cout<<"x: "<<x_min<<"  ->  "<<x_max<<endl;
    cout<<"y: "<<y_min<<"  ->  "<<y_max<<endl;
    cout<<"z: "<<z_min<<"  ->  "<<z_max<<endl;

    cout<<"cols : "<<sub_map_cols<<"   rows : "<<sub_map_rows<<"   tol_num  : "<<sub_map_num<<endl;

    point_cloud->points.clear();
    toc(t);

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
                    if(height>40) height = 40;
//                    o.r = jet_color_map[height*4+240][0];
//                    o.g = jet_color_map[height*4+240][1];
//                    o.b = jet_color_map[height*4+240][2];
                    o.r = jet_color_map[height*12+80][0];
                    o.g = jet_color_map[height*12+80][1];
                    o.b = jet_color_map[height*12+80][2];

//                    o.r = jet_color_map[height*8+160][0];
//                    o.g = jet_color_map[height*8+160][1];
//                    o.b = jet_color_map[height*8+160][2];

                    point_index++;
                    if(point_index%downsample == 0 && o.z > z_thresh){
                        point_cloud->points.push_back(o);
                    }

                }
        }
//        for(ColorOcTree::tree_iterator it = tree2.begin_tree(16),end=tree1.end_tree(); it!= end; ++it)
//        {
//            if (it.isLeaf())  //voxels for leaf nodes
//                if (tree1.isNodeOccupied(*it))  // occupied voxels
//                {
//                    //          if (tree.isNodeAtThreshold(*it)) {
//                    temp_point = it.getCoordinate();

//                    o.x=temp_point(0);
//                    o.y=temp_point(1);
//                    o.z=temp_point(2);

//                    if(stddev>0)
//                    {
//                        o.x += 1.0*dist(generator);
//                        o.y += 1.0*dist(generator);
//                        o.z += 1.0*dist(generator);
//                    }


//                    int height = o.z;

//                    height = (o.z+1)*10;
//                    if(height<0) height = 0;
//                    if(height>49) height = 49;
//                    o.r = jet_color_map[height*10][0];
//                    o.g = jet_color_map[height*10][1];
//                    o.b = jet_color_map[height*10][2];

//                    point_index++;
//                    if(point_index%downsample == 0 && o.z > z_thresh){
//                        point_cloud->points.push_back(o);
//                    }
//                }
//        }

    }
    else
    {
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

        for(int i=0;i<targets.size();i++)
        {
            o.x=targets[i].x;
            o.y=targets[i].y;
            o.z=targets[i].z;

            if(stddev>0)
            {
                o.x += 1.0*dist(generator);
                o.y += 1.0*dist(generator);
                o.z += 1.0*dist(generator);
            }

            int height;
            height = (o.z - z_mean + height_bias +1)*10;
            if(height<0) height = 0;
            if(height>40) height = 40;
            o.r = jet_color_map[height*4+240][0];
            o.g = jet_color_map[height*4+240][1];
            o.b = jet_color_map[height*4+240][2];

            point_index++;
            if(point_index%downsample == 0 && o.z > z_thresh)
                point_cloud->points.push_back(o);
        }
    }
    toc(t);

//    CreatSubMap(point_cloud);
    ShowMapCloud(point_cloud);
}

//-550.65 1101.3
bool plot_xy(float input_x, float input_y, bool sort_by_x, bool sort_by_y,
             int piece_num_all_x, int num_target_x,int piece_num_all_y, int num_target_y)
{
    if(sort_by_x && !sort_by_y)
    {
        float x_start = x_min + (num_target_x)*(x_max - x_min)/piece_num_all_x - 10;  //m
        float x_end = x_min + (num_target_x+1)*(x_max - x_min)/piece_num_all_x + 10;
        if(input_x>=x_start && input_x<=x_end)
            return true;
        else
            return false;
    }
    if(!sort_by_x && sort_by_y)
    {
        float y_start = y_min + (num_target_y)*(y_max - y_min)/piece_num_all_y - 10;
        float y_end = y_min + (num_target_y+1)*(y_max - y_min)/piece_num_all_y + 10;
        if(input_y>=y_start && input_y<=y_end)
            return true;
        else
            return false;
    }
    if(sort_by_x && sort_by_y)
    {
        float x_start = x_min + (num_target_x)*(x_max - x_min)/piece_num_all_x - 10;
        float x_end = x_min + (num_target_x+1)*(x_max - x_min)/piece_num_all_x + 10;
        float y_start = y_min + (num_target_y)*(y_max - y_min)/piece_num_all_y - 10;
        float y_end = y_min + (num_target_y+1)*(y_max - y_min)/piece_num_all_y + 10;
        if(input_y>=y_start && input_y<=y_end && input_x>=x_start && input_x<=x_end)
            return true;
        else
            return false;
    }
}

void CreatSubMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    octomap::OcTree* sub_map_tree (new octomap::OcTree( 0.1 ));
//    octomap::ColorOcTree sub_map_tree_color (new octomap::ColorOcTree( 0.1 ));
    pcl::PointXYZRGB o;

    FILE *fp_sub_map;
    int idx_map_x = 0, idx_map_y = 0;
    for( int rows_idx = 0; rows_idx<sub_map_rows; ++rows_idx ) {
        for( int cols_idx = 0; cols_idx<sub_map_cols; ++cols_idx ) {
            for( int point_idx = 0; point_idx < point_cloud->points.size(); ++point_idx ) {
                o.x =  point_cloud->points[point_idx].x;
                o.y =  point_cloud->points[point_idx].y;
                o.z =  point_cloud->points[point_idx].z;

                idx_map_x = (o.x - x_min) / SUB_MAP_X_SIZE;
                idx_map_y = (o.y - y_min) / SUB_MAP_Y_SIZE;
                if( idx_map_x == cols_idx && idx_map_y == rows_idx ) {
                    sub_map_tree->updateNode(octomap::point3d(o.x, o.y, o.z), true );
//                    sub_map_tree_color.updateNode()
                }

            }
            sub_map_tree->updateInnerOccupancy();
            std::string m_SavePath = "./octomap/submap_hexi_down/" ;
            std::string m_SaveFile = "./octomap/submap_hexi_down/" + std::to_string(rows_idx *sub_map_cols + cols_idx ) + ".ot";

            char  file_path[300];

            sprintf(file_path, m_SavePath.c_str());

            if(access(file_path,0)==-1)
                if(mkdir(file_path,0744)==-1)
                    std::cout<<"The data folder create error!"<<std::endl<<m_SavePath<<std::endl;

            sprintf(file_path, m_SaveFile.c_str());

            fp_sub_map = fopen(file_path, "w");
            sub_map_tree->write( file_path );
            sub_map_tree->clear();
            fclose(fp_sub_map);
            std::cout<<"got "<<rows_idx *sub_map_cols + cols_idx <<" sub map ."<<std::endl;

        }
    }
}

void ShowMapCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer ("map1!"));
    vis->addCoordinateSystem(1.0);
    vis->removeAllPointClouds();
    vis->removeAllShapes();
    vis->setBackgroundColor(0.0,0.0,0.0);
    vis->addPointCloud(point_cloud);
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1);
    vis->spin();
    vis->spinOnce(10);
}
