#include "../map/mapreadandwrite.h"
#include "../map/map_deal.h"
#include "../map/map_process.h"

const std::string map_frame_id="world";

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_map_server");
  ROS_INFO("package_name:robot_navigation  node_name:main_map_server");
  
  ros::NodeHandle n;
    // Latched publisher for data
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);  //原始地图
  ros::Publisher bmap_pub = n.advertise<nav_msgs::OccupancyGrid>("bmap", 1, true);
  ros::Publisher map_two_value_pub = n.advertise<nav_msgs::OccupancyGrid>("maptwovalue", 1, true);//代价地图

  nav_msgs::OccupancyGrid grid;
  nav_msgs::OccupancyGrid bmap;
  nav_msgs::OccupancyGrid map_two_value;  //二值化膨胀处理之后的网格地图

  const char * stem;
  if(argc>1)
    stem = argv[1];
  else
    stem= "/home/hulu/ros/expansion/vslam/catkin_ws/src/robot_navigation/2Dmap/zoulang_car_rtime_2D_map_sp8";
  ROS_INFO("2Dmap path: %s ",stem);
  if(! HL::PgmAndYamlToOccupancy(grid,stem))
    return -1;

  //by hulu add
  int  row = 0, col = 0, map_count = 0, k = 0;
  cv::String map_path = (cv::String)stem +".pgm";  
  uchar ** ptr = NULL;
  ptr = map_deal(&row,&col,map_path,&map_count); 
  if(ptr == NULL)
    return -1;
  ROS_INFO("row=%d  col=%d ",row,col);

  grid.info.map_load_time=ros::Time::now();
  grid.header.frame_id=map_frame_id;
  grid.header.stamp=ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %3.2lf m/cell",grid.info.width,
          grid.info.height,grid.info.resolution);


  unsigned long dat_size=grid.info.width*grid.info.height;
  bmap.header=grid.header;
  bmap.info=grid.info;
  bmap.data.resize(dat_size,-1);

  char *m=new char[dat_size];
  HL:: map_process(m,grid);

  for (size_t k = 0; k < dat_size; ++k) {
    bmap.data[k]=m[k];
  }

  delete m;

  //for two_value_map  by hulu  将二维数组数据转换为地图消息发送
  for (int i=0;i<row;i++)
  {
    for (int j=0;j<col;j++)
    {
      if(ptr[i][j]==1)	  //1为空闲删格
        ptr[i][j]=HL::kFreeGrid;
      else
        ptr[i][j]=HL::kOccGrid;
      //printf("%d ", ptr[i][j]);
    }
    //printf("\n");
  }

  map_two_value.header=grid.header;
  map_two_value.info=grid.info;
  map_two_value.data.resize(dat_size,-1); 

  // nav_msgs::OccupancyGrid中地图数据存储在rviz中显示第0行在rviz中最下面，故在此需要将ptr的数据复制给date需要行逆序赋值
  for (int i = grid.info.height-1; i >0; i--)
    for (int j = 0; j < grid.info.width; j++)
    {
    map_two_value.data[j + (grid.info.height-1-i) * grid.info.width] = ptr[i][j]; //origin down-left
    }

  map_pub.publish(grid);
  bmap_pub.publish(bmap);
  map_two_value_pub.publish(map_two_value);
  
  for(int i=0;i<row;i++)
    free(ptr[i]);
  free(ptr); 
  ros::spin();
  ros::shutdown();

  return 0;
}
