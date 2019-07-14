#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud, nav_msgs::Path, nav_msgs::Odometry> exact_policy;
//策略要求消息具有完全相同的时间戳以便匹配。 只有在具有相同确切时间戳的所有指定通道上收到消息时，才会调用回调

#include <surfel_map.h>

//by hulu add 
ros::Time currtimecount(2);
ros::Time lasttimecount(1);
bool no_topicget = false;
bool first_image = false;

void timerCallback(const ros::TimerEvent& e)  
{
    if(lasttimecount == currtimecount)
       no_topicget = true;
    lasttimecount = currtimecount;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surfel_fusion");
  ros::NodeHandle nh("~");

  SurfelMap surfel_map(nh);

  ros::Subscriber sub_image = nh.subscribe("image", 5000, &SurfelMap::image_input, &surfel_map);
  ros::Subscriber sub_depth = nh.subscribe("depth", 5000, &SurfelMap::depth_input, &surfel_map);
  ros::Subscriber sub_save_map = nh.subscribe("save_map", 5000, &SurfelMap::save_map, &surfel_map);

  /*ros::Subscriber sub_image = nh.subscribe("image", 5000,  &SurfelMap::left_image_sub, &surfel_map);
  ros::Subscriber sub_depth = nh.subscribe("depth", 5000, &SurfelMap::depth_sub, &surfel_map);
  ros::Subscriber sub_save_map = nh.subscribe("save_map", 5000, &SurfelMap::save_map, &surfel_map);*/

  // by hulu add 
  /*message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "image", 1000);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth", 1000);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync_image(sync_pol(1000), left_sub, depth_sub);
  sync_image.registerCallback(boost::bind(&SurfelMap::image_depth_input, &surfel_map,_1,_2));*/

  message_filters::Subscriber<sensor_msgs::PointCloud> sub_loop_stamps(nh, "loop_stamps", 1000);
  message_filters::Subscriber<nav_msgs::Path> sub_loop_path(nh, "loop_path", 1000);
  message_filters::Subscriber<nav_msgs::Odometry> sub_this_pose(nh, "this_pose", 1000);
  message_filters::Synchronizer<exact_policy> sync(exact_policy(1000), sub_loop_stamps, sub_loop_path, sub_this_pose);
  sync.registerCallback(boost::bind(&SurfelMap::orb_results_input, &surfel_map, _1, _2, _3));

  // ros::spin();

  //std::thread sync_thread(&SurfelMap::sync_process, &surfel_map);

  // ros::Rate r(20);
  ros::Timer timer = nh.createTimer(ros::Duration(8), timerCallback);//定时4s  by hulu
  while(ros::ok())
  {
    ros::spinOnce();
    if(no_topicget)
      break;
  }

  string save_name;
  if(nh.getParam("save_name", save_name))
  {
    string pcd_name = save_name + ".PCD";
    string mesh_name = save_name + "_mesh.PLY";
    string save_dense_PCD = save_name + "dense.PCD";
    surfel_map.save_cloud(pcd_name);
    surfel_map.save_mesh(mesh_name,save_dense_PCD);
  }

  return EXIT_SUCCESS;
}