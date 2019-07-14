/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<Eigen/Dense>
#include "../../../include/Converter.h"
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tf/transform_broadcaster.h>      //by hulu
#include <tf/transform_listener.h>         //by hulu  add 
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
using namespace ros;
using namespace Eigen;

ros::Publisher path_publish;
ros::Publisher pose_publish;
ros::Publisher loop_publish;
ros::Publisher pub_world_bf;         //by hulu  add 
ros::Publisher pub_path_world_bf;   //by hulu  add 
nav_msgs::Path path_world_bf;     //by hulu  add 
bool LastKeyframeDecision;
                                            //在命令行rosrun tf tf_echo  base_footprint mynteye_link_frame 获得
Vector3d T_bf_camera(0.362,0.003, 0.418);   //camera坐标系相对于base_footprint的固定平移  camera = myneye
bool isfirstpose = false;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void pub_world_bf_msg(Eigen::Matrix4d T_wc, ros::Time pubtime);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    bool first_image;
    int receive_counter;
    std::vector<ros::Time> receive_time_stamp;
    std::vector<std::pair<cv::Mat, cv::Mat>> loop_info;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);
    igb.receive_counter = 0;
    igb.first_image = false;

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1000);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    path_publish = nh.advertise<nav_msgs::Path>("orb_slam/path", 1000);
    pose_publish = nh.advertise<nav_msgs::Odometry>("orb_slam/pose", 1000);
    loop_publish = nh.advertise<sensor_msgs::PointCloud>("orb_slam/loop", 1000);
    pub_world_bf = nh.advertise<nav_msgs::Odometry>("orb_slam/world_bf_pose", 1000);
    pub_path_world_bf = nh.advertise<nav_msgs::Path>("orb_slam/path_world_bf", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    if(!first_image)
    {
        first_image = true;
        return;
    }

    receive_counter++;
    receive_time_stamp.push_back(msgLeft->header.stamp);

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat track_result;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        track_result = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        track_result = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    }

    while(mpSLAM->LocalMappingStopped())
    {
        void();
    }

    // transform the kitti pose
    /*static Eigen::Matrix4d transform_kitti;
    {       
        if(!isfirstpose)
        {
            Eigen::Matrix4d T_cw;
            for (int row_i = 0; row_i < 4; row_i++)
                for (int col_i = 0; col_i < 4; col_i++)
                    T_cw(row_i, col_i) = track_result.at<float>(row_i, col_i);   //track_result是一个世界坐标系相对于相机的位姿

            Eigen::Matrix4d idea_pose,idea_pose1;
            idea_pose = Eigen::Matrix4d::Zero();    
            idea_pose1 = Eigen::Matrix4d::Zero(); 
            idea_pose(0,2) = 1.0;      //将姿态绕世界坐标系y轴旋转90 右手坐标系
            idea_pose(1,1) = 1.0;  
            idea_pose(2,0) = -1.0;
            idea_pose(3,3) = 1.0;   
              
            idea_pose1(0,0) = 1.0;      //将姿态绕x轴旋转-90  右手坐标系
            idea_pose1(1,2) = 1.0;
            idea_pose1(2,1) = -1.0;
            idea_pose1(3,3) = 1.0;   
            transform_kitti = idea_pose1 * idea_pose * T_cw;

            isfirstpose = true;
        }
        //cout<<"transform_kitti = \n"<<transform_kitti<<endl;
    }*/
    // publish
    std::vector<std::pair<cv::Mat, double>> result_vector;
    mpSLAM->GetAllPoses(result_vector);
    nav_msgs::Path result_path;
    result_path.header.stamp = msgLeft->header.stamp;
    result_path.header.frame_id = "world";
    Eigen::Matrix4d temp_matrix, temp_matrix_inverse;
    Eigen::Matrix4d trans_form = Eigen::Matrix4d::Identity();  //Identity()为单位矩阵
    trans_form << 0,0,1,0, -1,0,0,0, 0,-1,0,0, 0,0,0,1;    //先绕y轴旋转90，在绕x轴旋转-90,之后的变换矩阵，右手坐标系
    for(int i = 0; i < result_vector.size(); i++)
    {
        geometry_msgs::PoseStamped this_pose;
        for (int j = 0; j < receive_time_stamp.size(); j++)
            if (fabs(receive_time_stamp[j].toSec() - result_vector[i].second) < 0.001)
            {
                this_pose.header.stamp = receive_time_stamp[j];
                break;
            }

        for(int row_i = 0; row_i < 4; row_i ++)
            for(int col_i = 0; col_i < 4; col_i ++)
                temp_matrix(row_i, col_i) = result_vector[i].first.at<float>(row_i, col_i);

        temp_matrix_inverse = trans_form * temp_matrix.inverse();   //获取的原位姿是世界相对于相机的，需求逆为相机相对于世界
                                                                  //原位姿是沿着z平移，且绕x旋转，需变换到沿着x移动，绕z旋转
        {
            //by hulu add 相机相对于车体中心有一个固定的平移
            Vector3d translate(temp_matrix_inverse(0, 3),temp_matrix_inverse(1, 3),temp_matrix_inverse(2, 3)); 
            translate = translate + T_bf_camera;
            temp_matrix_inverse(0, 3) = translate(0);
            temp_matrix_inverse(1, 3) = translate(1);
            temp_matrix_inverse(2, 3) = translate(2);
        }

        Eigen::Quaterniond rotation_q(temp_matrix_inverse.block<3, 3>(0, 0));
        this_pose.pose.position.x = temp_matrix_inverse(0, 3);
        this_pose.pose.position.y = temp_matrix_inverse(1, 3);
        this_pose.pose.position.z = temp_matrix_inverse(2, 3);
        this_pose.pose.orientation.x = rotation_q.x();
        this_pose.pose.orientation.y = rotation_q.y();
        this_pose.pose.orientation.z = rotation_q.z();
        this_pose.pose.orientation.w = rotation_q.w();
        result_path.poses.push_back(this_pose);
    }
    path_publish.publish(result_path);

    // get reference stamp
    double reference_stamp;
    reference_stamp = mpSLAM->GetRelativePose();
    int reference_index = 0;
    double time_diff = 1e9;    //1e9 = 10的9次方
    for (int i = 0; i < result_vector.size(); i++)
    {
        double this_time_diff = fabs(result_vector[i].second - reference_stamp);
        if (this_time_diff < time_diff)
        {
            reference_index = i;
            time_diff = this_time_diff;
        }
    }
    if (time_diff < 0.01)
        printf("the reference keyframe is %d, keyframe number %d.\n", reference_index, result_vector.size());
    else
        printf("cannot find the reference keyframe! time difference %f, the stamp is %f, current is %f.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
               time_diff,
               reference_stamp,
               msgLeft->header.stamp.toSec());

    // get keyframe decision
    LastKeyframeDecision = mpSLAM->GetKeyframeDecision();  //判断是否是关键帧
    if (LastKeyframeDecision)
        printf("this is keyframe.\n");

    nav_msgs::Odometry this_odometry;
    this_odometry.header.stamp = msgLeft->header.stamp;
    this_odometry.header.frame_id = "world";
    Eigen::Matrix4d T_cw, T_wc;
    for (int row_i = 0; row_i < 4; row_i++)
        for (int col_i = 0; col_i < 4; col_i++)
            T_cw(row_i, col_i) = track_result.at<float>(row_i, col_i);   //track_result是一个世界坐标系相对于相机的位姿
    T_wc = trans_form * T_cw.inverse();      //将获取到的相机姿态先绕y旋转90，在绕x旋转-90
    {
        //by hulu add 相机相对于车体中心有一个固定的平移
        Vector3d translate(T_wc(0, 3),T_wc(1, 3),T_wc(2, 3)); 
        translate = translate + T_bf_camera;
        T_wc(0, 3) = translate(0);
        T_wc(1, 3) = translate(1);
        T_wc(2, 3) = translate(2);
    }

    Eigen::Quaterniond rotation_q(T_wc.block<3, 3>(0, 0));
    this_odometry.pose.pose.position.x = T_wc(0, 3);
    this_odometry.pose.pose.position.y = T_wc(1, 3);
    this_odometry.pose.pose.position.z = T_wc(2, 3);
    this_odometry.pose.pose.orientation.x = rotation_q.x();
    this_odometry.pose.pose.orientation.y = rotation_q.y();
    this_odometry.pose.pose.orientation.z = rotation_q.z();
    this_odometry.pose.pose.orientation.w = rotation_q.w();
    if (LastKeyframeDecision)
        this_odometry.pose.covariance[0] = 1;
    else
        this_odometry.pose.covariance[0] = 0;
    this_odometry.pose.covariance[1] = reference_index;
    pose_publish.publish(this_odometry);

    // get loop index
    sensor_msgs::PointCloud ros_loop_info;
    ros_loop_info.header.stamp = msgLeft->header.stamp;
    ros_loop_info.header.frame_id = "this_is_loop_info";
    std::vector<std::pair<double, double>> loop_result;
    mpSLAM->GetLoopInfo(loop_result);
    sensor_msgs::ChannelFloat32 loop_channel;
    for (int i = 0; i < loop_result.size() && i < 35; i++)
    {
        int first_index = -1;
        int second_index = -1;
        for (int j = 0; j < result_vector.size(); j++)
        {
            if (result_vector[j].second == loop_result[i].first)
                first_index = j;
            if (result_vector[j].second == loop_result[i].second)
                second_index = j;
        }
        if (first_index > 0 && second_index > 0)
        {
            printf("the loop info %d <---> %d\n", first_index, second_index);
            loop_channel.values.push_back(first_index);
            loop_channel.values.push_back(second_index);
        }
        else
            printf("cannot find corresponding!\n");
    }
    ros_loop_info.channels.push_back(loop_channel);
    loop_publish.publish(ros_loop_info);

    pub_world_bf_msg(T_wc, msgLeft->header.stamp);
}

void ImageGrabber::pub_world_bf_msg(Eigen::Matrix4d T_wc, ros::Time pubtime)
{
    static tf::TransformBroadcaster br_world_camera,br_world_base_footprint;
    tf::Transform transform_world_camera,transform_world_bf;   //bf 为base_footprint
    tf::Quaternion q_world_camera,q_world_bf;

    //mynteye_left_frame -> base_footprint frame   by hulu  mynteye_left_frame = camera 坐标系为双目相机中心
    Vector3d translate_mynteye_bf(0.003, 0.418, -0.362);  //在命令行rosrun tf tf_echo mynteye_left_frame  base_footprint 获得
    Quaterniond rotation_mynteye_bf(0.499, 0.498, -0.501, 0.502);   //(w,x,y,z)

    //world -> mynteye 平移
    Vector3d T_temp(T_wc(0, 3),T_wc(1, 3),T_wc(2, 3));

    //world -> base_footprint frame   by hulu
    Vector3d translate_world_bf = T_temp + T_wc.block<3, 3>(0, 0) * translate_mynteye_bf;
    Quaterniond rotation_world_bf = Quaterniond((T_wc.block<3, 3>(0, 0)) * rotation_mynteye_bf.matrix());
    
    transform_world_bf.setOrigin(tf::Vector3(translate_world_bf(0),
                                    translate_world_bf(1),
                                    translate_world_bf(2)));
    q_world_bf.setW(rotation_world_bf.w());
    q_world_bf.setX(rotation_world_bf.x());
    q_world_bf.setY(rotation_world_bf.y());
    q_world_bf.setZ(rotation_world_bf.z());   
    transform_world_bf.setRotation(q_world_bf);    
    br_world_base_footprint.sendTransform(tf::StampedTransform(transform_world_bf, pubtime, "world", "base_footprint"));
    
    // world -> camera frame   by hulu  
    transform_world_camera.setOrigin(tf::Vector3(T_wc(0, 3),
                                    T_wc(1, 3),
                                    T_wc(2, 3)));
    Eigen::Quaterniond rotation_q(T_wc.block<3, 3>(0, 0));
    q_world_camera.setW(rotation_q.w());
    q_world_camera.setX(rotation_q.x());
    q_world_camera.setY(rotation_q.y());
    q_world_camera.setZ(rotation_q.z());
    transform_world_camera.setRotation(q_world_camera);
    br_world_camera.sendTransform(tf::StampedTransform(transform_world_camera, pubtime, "world", "camera"));

    //pub world -> base_footprint pose_topic
    nav_msgs::Odometry odometry;
    odometry.header.stamp = pubtime;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "base_footprint";
    odometry.pose.pose.position.x = translate_world_bf(0);
    odometry.pose.pose.position.y = translate_world_bf(1);
    odometry.pose.pose.position.z = translate_world_bf(2);
    odometry.pose.pose.orientation.x = rotation_world_bf.x();
    odometry.pose.pose.orientation.y = rotation_world_bf.y();
    odometry.pose.pose.orientation.z = rotation_world_bf.z();
    odometry.pose.pose.orientation.w = rotation_world_bf.w();
    pub_world_bf.publish(odometry);

    //pub world -> base_footprint path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = pubtime;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    path_world_bf.header.stamp = pubtime;
    path_world_bf.header.frame_id = "world";
    path_world_bf.poses.push_back(pose_stamped);
    pub_path_world_bf.publish(path_world_bf);
}