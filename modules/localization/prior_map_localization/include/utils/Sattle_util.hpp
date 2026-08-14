#ifndef __SATTLE_UTIL__
#define __SATTLE_UTIL__


#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "CoordOur.hpp"


using namespace std;

class Sattle_util
{
public:
    Sattle_util() {};

    void Init(string config_path_) {
        config_path = config_path_;
        LoadConfig();

        std::cout << "sattle path : " << mat_path << std::endl;

        sattle_mat = cv::imread(mat_path);
        cv::resize(sattle_mat, sattle_mat, cv::Size(), 0.2, 0.2, cv::INTER_LINEAR);

        mat_cols = sattle_mat.cols;
        mat_rows = sattle_mat.rows;

        resolution_lon = (sattle_corner_[2] - sattle_corner_[0]) / mat_cols;
        resolution_lat = (sattle_corner_[1] - sattle_corner_[3]) / mat_rows;
        std::cout<<"resolution_lon : "<< resolution_lon << std::endl;
        std::cout<<"resolution_lat : "<< resolution_lat << std::endl;
    }

    void LoadConfig() {

        YAML::Node config;
        try{
            config = YAML::LoadFile(config_path);
        }
        catch(YAML::BadFile &e){
            std::cerr << "read error config yaml !" << config_path <<std::endl;
        }

        map_center_    = config["init"]["map_center"].as<vector<double>>();
        sattle_corner_ = config["sattle"]["corner"].as<vector<double>>();
        mat_path       = config["sattle"]["mat_path"].as<string>("SatelliteMap/Baotou.jpg");

    }


    cv::Point2d Pixel2BLH(cv::Point2d pixel)
    {
        cv::Point2d blh; 
        blh.x = sattle_corner_[0] + resolution_lon * pixel.x;
        blh.y = sattle_corner_[1] - resolution_lat * pixel.y;
        return blh;
    }

    cv::Point2d Pixel2Gauss(cv::Point2d pixel)
    {
        cv::Point2d gauss;
        cv::Point2d blh = Pixel2BLH(pixel);
        XY_BLH::GaussProjCal(blh.x, blh.y, &gauss.x, &gauss.y);
        return gauss;
    }

    cv::Point2d BLH2Pixel(cv::Point2d blh)
    {
        cv::Point2d pixel;
        pixel.x = (blh.x - sattle_corner_[0]) / resolution_lon;
        pixel.y = (sattle_corner_[1] - blh.y) / resolution_lat;
        return pixel;
    }

    cv::Point2d Gauss2Pixel(cv::Point2d gauss)
    {
        cv::Point2d pixel;
        cv::Point2d blh;
        XY_BLH::GaussProjInvCal(gauss.x, gauss.y, &blh.x, &blh.y);
        pixel = BLH2Pixel(blh);
        return pixel;
    }



    string config_path;
    string mat_path;
    cv::Mat sattle_mat;
    double mat_cols, mat_rows;

    vector<double> map_center_;         // x, y, z
    vector<double> sattle_corner_;      // lon_tl, lat_tl, lon_br, lat_br

    double resolution_lon, resolution_lat;

};





#endif




