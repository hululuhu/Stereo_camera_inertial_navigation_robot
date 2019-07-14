# Stereo_camera_inertial_navigation_robot
基于双目视觉惯性的四轮差速自主导航机器人

启动顺序：//实时建立点云地图和面元地图
葫芦葫芦@-E6400：roccore
hulu @ hulu-E6400：cd / home / hulu / ros / expansion / vslam / Dashan_demo
hulu @ hulu-E6400：〜/ ros / expansion / vslam / Dashan_demo $ bash sh / telep.sh
hulu @ hulu-E6400：ssh 192.168.1.18 -l zhishan 
zhishan @ zhishan-robot：cd / home / zhishan / Dashan_demo_update
zhishan @ zhishan-robot：〜/ Dashan_demo_update $ bash sh / zhishanrobot_start.sh
hulu @hulu-E6400：〜/ ros / expansion / vslam / DenseSurfelMapping-HKUST / DenseSurfelMapping / ORB_SLAM2 $ ./orb_myneye_stereo_husac.sh
hulu @ hulu-E6400：〜/ ros / expansion / vslam / DenseSurfelMapping-HKUST / DenseSurfelMapping / catkin_ws $ roslaunch surfel_fusion myneye_orb_hustac.launch
hulu @ hulu-E6400：rosrun rviz rviz（/home/hulu/ros/expansion/vslam/DenseSurfelMapping-HKUST/DenseSurfelMapping/catkin_ws/src/surfel_fusion/surfel_map.rviz）
hulu @hulu-E6400：〜/ ros / expansion / vslam / MYNT-EYE-D-SDK_update / MYNT-EYE-D-SDK / wrappers / ros $ roslaunch mynteye_wrapper_d mynteye-hustac.launch //注意：必须先启动定位和建图，在启动相机，且相机驱动打开IR_depth_only

//实时建立八叉树地图与2D占据删格地图
按以上顺序启动定位，建图，小车节点，之后启动以下节点：
hulu @ hulu-E6400：〜/ ros / expansion / vslam / catkin_ws $ roslaunch pcdtodtpublic pcdconvert_tooctomap.launch //注意需要在rviz中添加octomap与2Dmap可视化的选项
hulu @ hulu-E6400：〜/ ros / expansion / vslam / catkin_ws / 3Dmap / ORB_SLAM2_maping / dateset $ rosrun octomap_server octomap_saver 524_rtime_octomap_sp8.bt //保存建立的实时octomap
hulu @ hulu-E6400：〜/ ros / expansion / vslam / catkin_ws / 3Dmap / ORB_SLAM2_maping / dateset $ rosrun map_server map_saver -f 524_2D_map_sp8 //保存2D占据删格地图
