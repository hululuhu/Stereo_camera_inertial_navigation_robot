#ifndef TRACKINGPTHREAD_H   
#define TRACKINGPTHREAD_H

#include <pthread.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Int32.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <termios.h>
#include "nav.h"

namespace HL 
{


  class TrackingPthread
  {
  private:
    pthread_t id;
    geometry_msgs::Twist vel;

  private:
    float maxForwardSpeed;
    float maxBackSpeed;
    float maxOmega;
    float maxUpAcc;
    float maxBackAcc;
    float poseChange;

    float maxDisErr;     //到达目标点的距离容差
    float fabsdfh;       ////转向的角度容差
    float maxAngErr;     //到达目标点的角度容差
    float PID[6];      //1~2为位置pid参数，3~5为航向角pid参数

    bool newGoal;   //导航控制参数
    bool startNav;  //开始导航
    bool emergeStop; //急停


  public:
    TrackingPthread();
    ~TrackingPthread();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void Stopcarcallback(const std_msgs::Int32::ConstPtr msg);
    void cmd_keyCallback(const geometry_msgs::Twist::ConstPtr & cmd);
    void orb_pose_input(const nav_msgs::OdometryConstPtr &pose);
    void PoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);
    void path_Callback(const sensor_msgs::PointCloudConstPtr& msgpoints);

    static void *MyPthread(void *temp);
    virtual void *DoPthread(void);
    void getNavcmd(void);
    void CalNavCmdVel(const NavPara *nav ,geometry_msgs::Twist& ctr);
    void initallnavstate(void);
  };

}
#endif // TRACKINGPTHREAD_H