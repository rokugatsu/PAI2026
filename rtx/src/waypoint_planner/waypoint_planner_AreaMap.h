#ifndef WAYPOINT_PLANNER_AREAMAP_H_
#define WAYPOINT_PLANNER_AREAMAP_H_

#pragma once
#include <fstream>
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

class AreaMapAccessor
{
public:

    void setMap(
        const nav_msgs::msg::OccupancyGrid& map);

    bool worldToGrid(
        double wx,
        double wy,
        int& gx,
        int& gy) const;

    // bool gridToWorld(
    //     int gx,
    //     int gy,
    //     double& wx,
    //     double& wy) const;

    int value(
        int gx,
        int gy) const;

    int valueWorld(
        double wx,
        double wy) const;

    // bool isTraversable(
    //     double wx,
    //     double wy) const;

    // bool isObstacle(
    //     double wx,
    //     double wy) const;

    void paintCircle(int gx,int gy,int radius);
    void paintTrajectory(double wx0,double wy0,double wx1,double wy1,double robot_width);
    bool savePGM(const std::string& pgm_file,const std::string& yaml_file) const;

    // 進行方向(yaw)に直交する方向の走行可能幅を計算する
    // wx, wy        : ロボット位置 [m]（world座標）
    // yaw           : ロボット進行方向 [rad]
    // max_half_width: 左右それぞれの最大探索距離 [m]
    // left, right   : 左/右の走行可能距離 [m]（出力）
    // width         : left + right [m]（出力）
    // allow_unknown : true なら未知領域(-1)も走行可能として扱う
    // 戻り値        : 開始位置が走行可能でなければ false
    bool traversableWidth(
        double wx,
        double wy,
        double yaw,
        double max_half_width,
        double& left,
        double& right,
        double& width,
        bool allow_unknown = false) const;

private:

    bool initialized_ = false;
    nav_msgs::msg::OccupancyGrid map_;

    bool isTraversableCell(int gx,int gy,bool allow_unknown) const;
    double rayDistance(
        double wx,double wy,
        double dir_x,double dir_y,
        double max_dist,
        bool allow_unknown) const;
};

#endif
