//#include "Localizer_node.hpp"

#pragma once

#include <fstream>
#include <sstream>   // （今後使うので入れておくと便利）

#include <string>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

enum AreaValue : uint8_t
{
    AREA_OBSTACLE      =   0,
    AREA_TEMPORARY     =  50,
    AREA_GRAVEL        = 100,
    AREA_UNKNOWN       = 205,//128,
    AREA_GRASS         = 150,
    AREA_CROSSWALK     = 180,
    AREA_SIDEWALK      = 200,
    AREA_PREFERRED     = 230,
    AREA_TRAVERSABLE   = 254//255
};

class AreaMapAccessor
{
public:

    bool load(
        const std::string& pgm,
        const std::string& yaml);

    bool load(
        const std::string& pgm_file);

    nav_msgs::msg::OccupancyGrid crop(double center_x,double center_y,double width,double height) const;

    nav_msgs::msg::OccupancyGrid
        occupancyGrid() const;

    bool initialized_ = false;
    bool worldToGrid(double wx,double wy,int& gx,int& gy) const;
    void paintTrajectory(double wx0,double wy0,double wx1,double wy1,double robot_width);
    //void paintCircle(int gx,int gy,int radius);
    void paintCircle(int gx,int gy,int radius,bool flag_force_paint);
    void inflateCircle(int cx,int cy,double radius);

    bool savePGM(const std::string& pgm_file,const std::string& yaml_file) const;

private:

    cv::Mat image_;

    nav_msgs::msg::OccupancyGrid occupancy_grid_;

    double resolution_;

    double origin_x_;

    double origin_y_;
};