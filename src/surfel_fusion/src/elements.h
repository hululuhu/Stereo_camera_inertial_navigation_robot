#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

struct Superpixel_seed
{
    float x, y;
    float size;   //超像素半径
    float norm_x, norm_y, norm_z;
    float posi_x, posi_y, posi_z;
    float view_cos;
    float mean_depth;
    float mean_intensity;
    bool fused;
    bool stable;

    // for debug
    float min_eigen_value;
    float max_eigen_value;
};

struct SurfelElement
{
    float px, py, pz;
    float nx, ny, nz;
    float size;   //面元半径
    float color;  //面元灰度值
    float weight;  //面元权重
    int update_times;  //面元更新时间
    int last_update;  //观察到该面元的上一个关键帧索引
};
