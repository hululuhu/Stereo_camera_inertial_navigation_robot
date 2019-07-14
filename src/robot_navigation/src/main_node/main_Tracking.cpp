#include <ros/ros.h>
#include "TrackingPthread.h"
#include <QCoreApplication>
#include <qdebug.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  QCoreApplication a(argc,argv);
  ros::init(argc, argv, "main_Tracking");
  ROS_INFO("package_name:robot_navigation  node_name:main_Tracking");
  qDebug()<<"qDebug ok!";

  HL::TrackingPthread TrackingPthread;
 
  a.exec();
  ros::shutdown();
  return 0;
}


