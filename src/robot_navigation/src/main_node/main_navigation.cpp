#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int32.h"
#include "nav.h"
#include "../route_plan_algorithm/A_star.h"
#include "../map/map_process.h"
#include "../map/map_deal.h"

static HL::NavPara m_navPara = {{0, 0, 0}, {0, 0, 0}, false, false};
static HL::CarPose last_current = {0,0,0};
static bool getPath = false;            //获得全局路径标志位
static bool hasNewPose = false;
static bool localgetPath = false;       //获得局部路径标志位
static bool rviznewgoal_flag=false;     //获得rviz设定的全局目标点标志
static bool readmapfinish_flag=false;   //获得全局地图信息标志
static bool readsubmapfinish_flag=false; //获得局部子图信息标志
static int map_height=0;
static int map_width=0;
static float map_resolution=0.0;
static float map_frame_originx=0.0;
static float map_frame_originy=0.0;
static sensor_msgs::PointCloud orginaStartPath;  //存储初始规划路径点
static sensor_msgs::PointCloud ValidaStartPath;  //存储优化后的关键路径点
static sensor_msgs::PointCloud ValidgloabPath_printf;  //显示优化后的路径点
static sensor_msgs::PointCloud Trajectory_printf;  //显示运动轨迹点
static std::vector<std::vector<char>> maze;      
static float poseChangePow2=0;//0.025*0.025;   //移动大于10cm才会更新子图数组   20180918改
static float maxDisErr=0.30;                  //20180918改
static int submap_height=0;
static int submap_width=0;
static float submap_resolution=0.0;
static sensor_msgs::PointCloud orginlocalPath;  //存储局部初始规划路径点
static sensor_msgs::PointCloud ValidlocalPath;  //存储局部优化后的路径点
static float Avoidancedistance = 0.0; //运动过的距离

static uchar ** ptr = NULL;
static nav_msgs::OccupancyGrid map_two_value;  //二值化膨胀处理之后的网格地图
static std::vector<std::vector<char>> maze_gloab_map;  
static std::vector<std::vector<char>> map_init_state;   //用于存储初始全局代价地图信息
static bool timerout=false;

void rvizgoal_Callback(const geometry_msgs::PoseStamped& msgPose); //订阅rviz设定的目标点消息
void PoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);  //订阅定位节点发布的位姿消息
void orb_pose_input(const nav_msgs::OdometryConstPtr &pose);  //订阅orb_slam2的的定位位姿话题
void SubMapReceived(const nav_msgs::OccupancyGridConstPtr &msg); //订阅定位节点发布的局部代价地图消息
void MapReceived(const nav_msgs::OccupancyGridConstPtr &msg);    //订阅地图服务器节点发布的全局代价地图消息
int  A_starplan(uchar ** ptr,int row,int col,int map_count);      //全局A*路径规划  
int  local_Astarplan(int row,int col,int map_count,float submap_resolution,HL::CarPose localdesired);  //局部A*路径规划  暂时未用到
bool isValidlocalLine(const std::vector<std::vector<char>> &maze, int x1, int y1, int x2, int y2);    //局部路径优化函数  暂时未用到
bool isValidLine( uchar ** ptr, int x1, int y1, int x2, int y2);  //全局路径优化函数
void timerCallback(const ros::TimerEvent& event);      //定时器回调函数，用于定时清除全局代价地图引入的新障碍物
bool judge_localdesiredisoccd(HL::CarPose localdesired);   //判断局部目标点与局部起始点之间的直线上是否有障碍物
void gloabcostmap_clear(bool gloabplanfile);               //清除全局代价地图上的后期添加的障碍物
bool judgment_neighbors(int *rows_current_x, int *cols_current_y,int row,int col,int range);   //判断当前点周围8个方向范围内是否是有空闲区域
void ValidPath_printf(pAStarNode *path_stack,int Validnewstartindex,int iflag);  //打印优化后的路径

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main_navigation");
    ROS_INFO("package_name:robot_navigation  node_name:main_navigation");
    ros::NodeHandle n;
	int  row = 0, col = 0, map_count = 0;
    bool start_nav=false; 
    
    ros::Subscriber navigation_goal_sub = n.subscribe("/move_base_simple/goal", 2, rvizgoal_Callback);   //by hulu  订阅目标位姿
    ros::Subscriber orbslam2_pose_sub = n.subscribe<nav_msgs::Odometry>("/orb_slam/world_bf_pose", 2, orb_pose_input);  //订阅orbslam2定位位姿
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, PoseReceived);  //订阅amcl定位位姿
    ros::Subscriber map_sub = n.subscribe("maptwovalue", 2, MapReceived);   //by hulu 订阅二值化、膨胀之后的图像数组数据，并发布话题maptwovalue
    ros::Subscriber submap_sub = n.subscribe("subMap", 2, SubMapReceived);  //订阅局部代价地图

    ros::Publisher gloab_orginpath = n.advertise<sensor_msgs::PointCloud>("gloab_orginpath", 1, true); //发布用于rviz显示的实际规划的路径点
    ros::Publisher gloab_Validpath = n.advertise<sensor_msgs::PointCloud>("gloab_Validpath", 1, true); //发布用于导航控制的优化路径点
    ros::Publisher gloab_Validpath_printf = n.advertise<sensor_msgs::PointCloud>("gloabValidpath_printf", 1, true); //发布显示优化后的路径点
    ros::Publisher Trajectory_printf_pub = n.advertise<sensor_msgs::PointCloud>("Trajectory_printf", 1, true); //发布显示运动轨迹
    //ros::Publisher local_orginpath = n.advertise<sensor_msgs::PointCloud>("local_orginpath", 1, true); //发布用于rviz显示的局部实际规划的路径点
    //ros::Publisher local_Validpath = n.advertise<sensor_msgs::PointCloud>("local_Validpath", 1, true); //发布用于导航控制的局部优化路径点

    ros::Publisher map_two_value_pub = n.advertise<nav_msgs::OccupancyGrid>("maptwovalue_test", 1, true);  //发布通过局部代价地图引入障碍物的全局代价地图
    ros::Publisher stopcar_pub = n.advertise<std_msgs::Int32>("main_navigation/Stopcar", 10, true);   //发布停车/发车指令

    cv::String map_path;
    if(argc>1)
      map_path = (cv::String)argv[1];
    else
      map_path= "/home/hulu/ros/expansion/vslam/catkin_ws/src/robot_navigation/2Dmap/zoulang_car_rtime_2D_map_sp8.pgm";
    
	ptr = map_deal(&row,&col,map_path,&map_count); //处理全局.pgm地图，二值化并膨胀
    ROS_INFO("2Dmap read down! row=%d,cols=%d",row,col);
    ROS_INFO("loop start please input gloab target to path plan!");
    
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback,false);  //false表示连续定时
    ros::Rate loop_rate(50);   //20ms 循环
    while (ros::ok())
    {
        ros::spinOnce();   //使各个回调函数发生
        
        if(hasNewPose)
        {
            geometry_msgs::Point32 pa;
            pa.x = m_navPara.current.x;
            pa.y = m_navPara.current.y;
            pa.z = 0;
            Trajectory_printf.points.push_back(pa); //该点云结构体存储的是机器人运动轨迹

            Trajectory_printf.header.frame_id = "world";
            Trajectory_printf.header.stamp = ros::Time::now();
            Trajectory_printf_pub.publish(Trajectory_printf);

            float dx = m_navPara.current.x - last_current.x;
            float dy = m_navPara.current.y - last_current.y;
            Avoidancedistance+=sqrt(dx*dx+dy*dy);
            //printf("当前经过的距离:Avoidancedistance%6.2f \n",Avoidancedistance);

            hasNewPose = false;
        }

        if(timerout && readmapfinish_flag)    //定时清除全局代价地图中新引入的障碍物
        {
            gloabcostmap_clear(false);
            map_two_value_pub.publish(map_two_value);

            timerout=false;
            //ROS_INFO("timerout");
        }

        if(rviznewgoal_flag && readmapfinish_flag)  //订阅到全局目标点，并已经订阅到全局代价地图，则进行全局路径规划
        {
            A_starplan(ptr,row,col,map_count);
            rviznewgoal_flag = false;

            /*int size = ValidaStartPath.points.size();
            //计算优化后的路径长度
            float distance = 0;
            float dx = ValidaStartPath.points[0].x - m_navPara.current.x;
            float dy = ValidaStartPath.points[0].y - m_navPara.current.y;
            distance+=sqrt(dx*dx+dy*dy);
            for(int i=0;i<size-1;i++)
            {
                dx= ValidaStartPath.points[i].x - ValidaStartPath.points[i+1].x;
                dy= ValidaStartPath.points[i].y - ValidaStartPath.points[i+1].y;
                distance+=sqrt(dx*dx+dy*dy);
            }
            printf("当前优化路径总长:Valid_distance为%6.2f \n",distance);

            //计算原始路径长度
            float orgindistance=0;
            int orginsize = orginaStartPath.points.size();
            for(int i=0;i<orginsize-1;i++)
            {
                float dx = orginaStartPath.points[i].x - orginaStartPath.points[i+1].x;
                float dy = orginaStartPath.points[i].y - orginaStartPath.points[i+1].y;
                orgindistance+=sqrt(dx*dx+dy*dy);
            }
            printf("当前原始路径总长:Valid_distance为%6.2f \n",orgindistance);*/
        }
       
        if(getPath)   //发布规划好的全局路径与优化后的全局路径
       {
           {
                getPath = false;
                orginaStartPath.header.frame_id = "world";
                orginaStartPath.header.stamp = ros::Time::now();
                gloab_orginpath.publish(orginaStartPath);

                ValidaStartPath.header.frame_id = "world";
                ValidaStartPath.header.stamp = ros::Time::now();
                gloab_Validpath.publish(ValidaStartPath);

                ValidgloabPath_printf.header.frame_id = "world";
                ValidgloabPath_printf.header.stamp = ros::Time::now();
                gloab_Validpath_printf.publish(ValidgloabPath_printf);

                if(start_nav)     //为了后续全局路径规划之后自动启动小车
                {
                    std_msgs::Int32 startcar;
                    startcar.data=1;
                    stopcar_pub.publish(startcar);
                    start_nav = false;
                }
            }
       }
      
       //用于判断是否需要重新规划全局路径
       if(readsubmapfinish_flag)
       {
           if(ValidaStartPath.points.size()) 
           {
               int size = ValidaStartPath.points.size();    
               static int iflag = 0;
               static char meetoccdcount=0;     //局部目标点与局部起始点之间遇到障碍物的次数
               static bool gloabplanfail=false;
               static HL::CarPose nextDest={0,0,0};

               if(iflag<size)
               {
                    nextDest.x=ValidaStartPath.points[iflag].x;
                    nextDest.y=ValidaStartPath.points[iflag].y;
                    nextDest.h=ValidaStartPath.points[iflag].z;

                    float dx, dy, ds;
                    dx = nextDest.x - m_navPara.current.x;
                    dy = nextDest.y - m_navPara.current.y;
                    ds = sqrt(dx * dx + dy * dy);
        
                    if(fabs(ds) < maxDisErr)   //这儿很关键，maxDisErr设置过小造成躲避障碍物反应迟钝，可能会撞到障碍物
                        iflag++;
                    else 
                    {
                        bool isoccd_localdaim=judge_localdesiredisoccd(nextDest);  //判断局部目标点与局部起始点之间是否存在障碍物
                        if(isoccd_localdaim)    //存在障碍物
                        {
                            meetoccdcount++;
                            if(meetoccdcount>=4)    //若遇到障碍物的次数大于3次，则发送停车指令，并再次规划路径 ,防止误检
                            {
                                std_msgs::Int32 stopcar;
                                stopcar.data=2;                 //1为发车，2为停车
                                stopcar_pub.publish(stopcar);   //发送停车指令话题

                                A_starplan(ptr,row,col,map_count);

                                if(ValidaStartPath.points.size()==0) //路径规划再次失败
                                {
                                    gloabplanfail=true;   //路径规划失败，复位全局代价地图
                                    gloabcostmap_clear(gloabplanfail);
                                    A_starplan(ptr,row,col,map_count);
                                }
       
                                if(getPath)  //找到路径，设置开始导航标志
                                {
                                    start_nav = true;
                                }

                                iflag=0;
                                meetoccdcount=0;
                                gloabplanfail=false;
                            }
                        }
                        else ;
                    }
                }
               else
               {
                    iflag = 0;
                    nextDest = m_navPara.current;
                    orginaStartPath.points.clear();
                    ValidaStartPath.points.clear();
                }
            }
            else ; 
            readsubmapfinish_flag = false;
        }

      loop_rate.sleep();   //为了让定时发生注释
    }
    ros::shutdown();
	return 0;
}

//rviz设定目标位姿回调函数
void rvizgoal_Callback(const geometry_msgs::PoseStamped& msgPose)
{
   m_navPara.desired.x =(float) msgPose.pose.position.x;
   m_navPara.desired.y =(float) msgPose.pose.position.y;
   m_navPara.desired.h =(float)(tf::getYaw(msgPose.pose.orientation));
   printf("rviz设定的目标点为[%6.2f,%6.2f,%6.2f]\n",m_navPara.desired.x,m_navPara.desired.y,m_navPara.desired.h);
   rviznewgoal_flag=true;
}

//定位位姿回调函数
void PoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose)
{
    last_current = m_navPara.current;
    m_navPara.current.x = (float)pose->pose.pose.position.x;
    m_navPara.current.y = (float)pose->pose.pose.position.y;
    m_navPara.current.h = (float)(tf::getYaw(pose->pose.pose.orientation));

    hasNewPose = true;
}

void orb_pose_input(const nav_msgs::OdometryConstPtr &pose)
{
    last_current = m_navPara.current;
    m_navPara.current.x = (float)pose->pose.pose.position.x;
    m_navPara.current.y = (float)pose->pose.pose.position.y;
    m_navPara.current.h = (float)(tf::getYaw(pose->pose.pose.orientation));

    hasNewPose = true;
}

//全局代价地图回调函数，只回调一次
void MapReceived(const nav_msgs::OccupancyGridConstPtr &msg)
{
    map_height=msg->info.height;
    map_width=msg->info.width;
    map_resolution=msg->info.resolution;
    map_frame_originx=msg->info.origin.position.x;
    map_frame_originy=msg->info.origin.position.y;

    unsigned long dat_size=msg->info.width*msg->info.height;
    map_two_value.header=msg->header;
    map_two_value.info=msg->info;
    map_two_value.data.resize(dat_size,-1); 

    maze_gloab_map.clear();
    map_init_state.clear();
    maze_gloab_map.resize(msg->info.height, std::vector<char>(msg->info.width, -1));  //用-1初始化容器值
    map_init_state.resize(msg->info.height, std::vector<char>(msg->info.width, -1));  //用-1初始化容器值

    for(int i = 0 ; i < msg->info.height; i++)
        for(int j = 0; j < msg->info.width; j++)
        {
            maze_gloab_map[i][j] = msg->data[j + (msg->info.height-1-i) * msg->info.width]; //origin down-left
            map_init_state[i][j] = msg->data[j + (msg->info.height-1-i) * msg->info.width]; //存储地图初始状态
            //ROS_INFO("[ %d ]",maze_gloab_map[i][j]);
            //maze_gloab_map[i][j] = msg->data[j + i * msg->info.width];
       }

    readmapfinish_flag=true;
}

//定时器回调函数
void timerCallback(const ros::TimerEvent& event)
{
    timerout=true;
}

//订阅局部子图消息
void SubMapReceived(const nav_msgs::OccupancyGridConstPtr &msg)
{
    static HL::CarPose lastCarPose={0,0,0};
    float poseChangePow = (lastCarPose.x-m_navPara.current.x)*(lastCarPose.x-m_navPara.current.x) +
        (lastCarPose.y-m_navPara.current.y)*(lastCarPose.y-m_navPara.current.y);

    submap_height=msg->info.height;
    submap_width=msg->info.width;
    submap_resolution=msg->info.resolution;

    if(poseChangePow > poseChangePow2)
    {
        lastCarPose = m_navPara.current;
        readsubmapfinish_flag=true;
    
        //printf("sub_map height:%d sub_map width:%d \n",msg->info.height,msg->info.width);
        int submap_center_x = submap_height / 2;  //子图中心坐标
        int submap_center_y = submap_width / 2;

        maze.clear();
        maze.resize(msg->info.height, std::vector<char>(msg->info.width, -1));  //用-1初始化容器值
        for (int i = 0; i < msg->info.height; i++)
            for (int j = 0; j < msg->info.width; j++)
            {
                maze[i][j] = msg->data[j + i * msg->info.width]; //origin up-left

                geometry_msgs::Point32 submapoccdxy;   //局部子图相对于map坐标系的实际位置
                submapoccdxy.y = (i - submap_center_x) * submap_resolution + m_navPara.current.y;
                submapoccdxy.x = (j - submap_center_y) * submap_resolution + m_navPara.current.x; 

                //将局部子图相对于map坐标系的世界位置转换到像素坐标点
                int occd_map_row= (int)( map_height - (submapoccdxy.y -  map_frame_originy)/map_resolution );
                int occd_map_col= (int)((submapoccdxy.x - map_frame_originx)/map_resolution );

                if(maze[i][j]==HL::kOccGrid)
                {
                    maze[i][j]=BARRIER;     //障碍物是0
                    
                    if(readmapfinish_flag)    //将检测到的局部障碍物添加到全局代价地图
                    {
                        ptr[occd_map_row][occd_map_col]=BARRIER;
                        maze_gloab_map[occd_map_row][occd_map_col]=HL::kOccGrid;
                    }
                }  
                else
                {
                    maze[i][j]=free_grid;    //空闲是1

                    if(readmapfinish_flag)    //若当前检测到的点为自由点，则清除之前全局代价地图中添加的障碍物
                    {   
                        if(map_init_state[occd_map_row][occd_map_col]==HL::kFreeGrid)
                        {
                            ptr[occd_map_row][occd_map_col]=free_grid;
                            maze_gloab_map[occd_map_row][occd_map_col]=HL::kFreeGrid;
                        }
                        else ;
                    }
                }
                //printf("%d ", maze[i][j]);
            }
            //printf("\n");
        //readsubmapfinish_flag=true;
    }
}

//清除由局部代价地图建立在全局代价地图中的障碍物
void gloabcostmap_clear(bool gloabplanfile)
{
    int submap_center_y = submap_height / 2;  //子图中心坐标
    int submap_center_x = submap_width / 2;

    geometry_msgs::Point32 submapstartxy;   //局部子图起始点相对于map坐标系的实际位置  即局部子图的左上角
    geometry_msgs::Point32 submapendxy;   //局部子图终点相对于map坐标系的实际位置      即局部子图的右下角

    submapstartxy.y = (0 - submap_center_y) * submap_resolution + m_navPara.current.y;
    submapstartxy.x = (0 - submap_center_x) * submap_resolution + m_navPara.current.x; 

    submapendxy.y = (submap_height - submap_center_y) * submap_resolution + m_navPara.current.y;
    submapendxy.x = (submap_width - submap_center_x) * submap_resolution + m_navPara.current.x; 

    //将局部子图相对于map坐标系的世界位置转换到全局地图像素坐标点
    int submapstart_row= (int)( map_height - (submapstartxy.y - map_frame_originy)/map_resolution );
    int submapstart_col= (int)((submapstartxy.x - map_frame_originx)/map_resolution );

    int submapend_row= (int)( map_height - (submapendxy.y -  map_frame_originy)/map_resolution );
    int submapend_col= (int)((submapendxy.x - map_frame_originx)/map_resolution );

    for (int i = map_height-1; i>0; i--)
        for (int j = 0; j < map_width; j++)
        {
            if((i<=submapstart_row && i>=submapend_row) && (j>=submapstart_col && j<=submapend_col))   //该子图范围内的像素点属性不会被复位
            {
                if(gloabplanfile)     //目的是用于在全局路径规划失败的情况下，复位整个全局代价地图（包括局部子图范围内）回到起始状态，再次规划全局路径
                {
                    if(map_init_state[i][j]==HL::kOccGrid)   //目的是为了将障碍物设置为BARRIER，自由空间设置为free_grid
                        ptr[i][j]=BARRIER;
                    else
                        ptr[i][j]=free_grid;
                
                    maze_gloab_map[i][j]=map_init_state[i][j];
                }

                map_two_value.data[j + (map_height-1-i) * map_width] = maze_gloab_map[i][j]; //origin down-left
                continue;
            }
            else
            {
                if(map_init_state[i][j]==HL::kOccGrid)   //目的是为了将障碍物设置为BARRIER，自由空间设置为free_grid
                    ptr[i][j]=BARRIER;
                else
                    ptr[i][j]=free_grid;
                
                maze_gloab_map[i][j]=map_init_state[i][j];
                map_two_value.data[j + (map_height-1-i) * map_width] = maze_gloab_map[i][j]; //origin down-left
            }
        }
        
}

bool judge_localdesiredisoccd(HL::CarPose localdesired)
{
    int row=submap_height;
    int col=submap_width;

    float submap_fx = localdesired.x - m_navPara.current.x;  //相对于机器人当前位置的距离值
    float submap_fy = localdesired.y - m_navPara.current.y;

    float boundx = (col - 1) * submap_resolution * 0.5;   //为什么要*0.5 子图长宽的一半
    float boundy = (row - 1) * submap_resolution * 0.5;  //bound，子图边界
    float fk = (boundy / boundx);                     //斜率，子图高的一半比上宽的一半

    HL::CarPose tempGoal{submap_fx, submap_fy, localdesired.h};

    if ((fabs(submap_fx) > boundx) || (fabs(submap_fy) > boundy))  //将目标移动距离限制在子图范围内
    {
        localdesired.h=0;   //若当前目标点超出submap范围，将子图目标点航向角设定为０度；
        if (submap_fx == 0)
        {
            tempGoal.x = 0;
            tempGoal.y = ((submap_fy >= 0) ? boundy : -boundy);
        }
        else if (submap_fy == 0)
        {
            tempGoal.x = ((submap_fx >= 0) ? boundx : -boundx);
            tempGoal.y = 0;
        }
        else
        {
            float k = submap_fy / submap_fx;
            if (fabs(k) <= fk)
            {
                tempGoal.x = ((submap_fx >= 0) ? boundx : -boundx);
                tempGoal.y = k * tempGoal.x;     //根据公式得到另一个坐标,tan*x=y
            }
            else
            {
                tempGoal.y = ((submap_fy >= 0) ? boundy : -boundy);
                tempGoal.x = tempGoal.y / k;
            }
        }
    } //转换为局部子图中的局部目标，相对于子图中心坐标系
    int localend_y = floor(tempGoal.x / submap_resolution + 0.5) + col / 2;  //将实际距离转换为地图中的像素点位置
    int localend_x = floor(tempGoal.y / submap_resolution + 0.5) + row / 2; //floor向下取整，取不大于floor(x)的整数

    int localstart_x = row / 2;  //机器人所在的子图中心为当前起始点
    int localstart_y = col / 2;
  
    //用于打印调试
    /*ROS_INFO("current_x=%6.2f,current_y=%6.2f",m_navPara.current.x,m_navPara.current.y);
    ROS_INFO("localend_x=%6.2f,localend_y=%6.2f",tempGoal.x,tempGoal.y);
    ROS_INFO("localstart_x=%d,localstart_y=%d",localstart_x,localstart_y);
    ROS_INFO("localend_x=%d,localend_y=%d",localend_x,localend_y);*/

    //判断当前局部目标点与局部起始点之间的直线上是否存在障碍物   返回false则有障碍物
    bool localisoccd=isValidlocalLine(maze,localend_x,localend_y, localstart_x, localstart_y);

    if(!localisoccd)  
    {
        //printf("局部目标点与起始点之间有障碍物，停止，重新规划全局路径\n");  //用于调试
        return true;
    }
    else
    {
        //printf("局部目标点与起始点之间没有障碍物\n");     //用于调试
        return false;
    }  
}

//全局A*路径规划
int  A_starplan(uchar ** ptr,int row,int col,int map_count)
{
    orginaStartPath.points.clear();
    ValidaStartPath.points.clear();
    ValidgloabPath_printf.points.clear();
    //将相对于map坐标系的坐标位置转换为地图中的像素点位置
    int rows_desired_x= (int)( map_height - (m_navPara.desired.y -  map_frame_originy)/map_resolution );
    int cols_desired_y= (int)((m_navPara.desired.x - map_frame_originx)/map_resolution );
       
    int rows_current_x=(int)( map_height - (m_navPara.current.y -  map_frame_originy)/map_resolution );
    int cols_current_y=(int)((m_navPara.current.x - map_frame_originx)/map_resolution );

    ROS_INFO("current_x=%6.2f,current_y=%6.2f",m_navPara.current.x,m_navPara.current.y);
    ROS_INFO("desired_x=%6.2f,desired_y=%6.2f",m_navPara.desired.x,m_navPara.desired.y);
    ROS_INFO("rows_current_x=%d,cols_current_y=%d",rows_current_x,cols_current_y);
    ROS_INFO("rows_desired_x=%d,cols_desired_y=%d",rows_desired_x,cols_desired_y);

    if( rows_desired_x<0 || rows_desired_x >= map_height || cols_desired_y< 0 || cols_desired_y >= map_width)
    {
       printf("目标点超出范围,路径规划失败\n");
       return 1;   //路径规划失败，返回1
    }
    if (rows_current_x == rows_desired_x && cols_current_y == cols_desired_y)
    {
        printf("当前点与目标点重合，路径规划失败\n");
        return 1;
    }
    if (ptr[rows_desired_x][cols_desired_y] == BARRIER)  //判断全局目标点是否是障碍物
    {
        printf("目标点为障碍物,路径规划失败\n");
        return 1;
    }
    if (ptr[rows_current_x][cols_current_y] == BARRIER)  //判断全局起始点是否是障碍物
    {
        printf("起始点为障碍物,路径规划失败\n");
        if(judgment_neighbors(&rows_current_x,&cols_current_y,row,col,8))   //判断起始点周围8个方向8个像素范围内是否存在障碍物 
            printf("寻找到新的起始点,继续路径规划\n");                           //8这个参数很关键，若设置得过小，无法找到新的起始点
        else
        {
            printf("寻找起始点失败,路径规划失败\n");
            return 1;
        }
    }
    printf("目标点设置正确,进行路径规划\n");

    int init_desired_state = ptr[rows_desired_x][cols_desired_y];
    int init_current_state = ptr[rows_current_x][cols_current_y];
    ptr[rows_current_x][cols_current_y]=STARTNODE;          //在地图中标识起始点和目标点
	ptr[rows_desired_x][cols_desired_y]=ENDNODE;

    map_maze = (AStarNode  **)malloc(row * sizeof(AStarNode  *));  
	for (int k=0;k<row;k++)
		map_maze[k] = (AStarNode  *)malloc(col * sizeof(AStarNode));

    //注意，这儿的分配内存有点浪费，需要改为动态的
	open_table  = (pAStarNode *)malloc(map_count * sizeof(pAStarNode));  
	close_table = (pAStarNode *)malloc(map_count * sizeof(pAStarNode)); 
	path_stack  = (pAStarNode *)malloc(map_count * sizeof(pAStarNode));  
	//path_stack_deal = (pAStarNode *)malloc(map_count * sizeof(pAStarNode));  

	AStarNode *start_node=NULL;		
	AStarNode *end_node= NULL;		
	AStarNode *curr_node= NULL;			

	int    is_found = 0;		 //找到路径标志位

	for (int i=0;i<row;++i)
	{
		for (int j=0; j<col;++j)
		{
			map_maze[i][j].s_g = 0;
			map_maze[i][j].s_h = 0;
			map_maze[i][j].s_is_in_closetable = 0;
			map_maze[i][j].s_is_in_opentable = 0;
			map_maze[i][j].s_style = ptr[i][j];
			map_maze[i][j].s_x = i;
			map_maze[i][j].s_y = j;
			map_maze[i][j].s_parent = NULL;

			if (map_maze[i][j].s_style==STARTNODE)	// 起点
			{
				start_node=&(map_maze[i][j]);
			}
			else if (map_maze[i][j].s_style==ENDNODE)	// 终点
			{
				end_node=&(map_maze[i][j]);
			}
            else ;
			//printf("%d ", maze[i][j]);
		}
		//printf("\n");
	}

	open_table[open_node_count++] = start_node;			
	start_node->s_is_in_opentable = 1;			
	start_node->s_g = 0;
	start_node->s_h = abs(end_node->s_x - start_node->s_x) + abs(end_node->s_y - start_node->s_y);
	//start_node->s_h = (int)round(sqrt((end_node->s_x - start_node->s_x) *(end_node->s_x - start_node->s_x) + (end_node->s_y - start_node->s_y) * (end_node->s_y - start_node->s_y)));

	start_node->s_parent = NULL;
	if (start_node->s_x==end_node->s_x && start_node->s_y==end_node->s_y)
	{
        printf("起点==终点!\n");
		return 1;
	}

	while (1)
	{
		curr_node=open_table[0];		// open表的第一个点一定是f值最小的节点(通过堆排序得到的)
		open_table[0]=open_table[--open_node_count];	// 最后一个点放到open表的第一个点，然后进堆调整 此时已经在open表中删除当前节点 
		adjust_heap(0);				// 堆调整

		close_table[close_node_count++]=curr_node;	// 当前点加入到close表
		curr_node->s_is_in_closetable=1;		// 已经在close表中

		if (curr_node->s_x==end_node->s_x && curr_node->s_y==end_node->s_y)// 终点在close表，结束规划
		{
			is_found=1;
			break;
		}

		get_neighbors(curr_node, end_node,row,col);			// 对邻居的处理

		if (open_node_count==0)				// 没有找到路径点
		{
			is_found=0;
			break;
		}
	}

	//显示搜索到的最优路径
	if (is_found)
	{
		curr_node=end_node;
		while (curr_node)      //top=0为目标点
		{
			path_stack[++top]=curr_node;
			curr_node=curr_node->s_parent;
		}
		//处理规划后的路径及在地图中输出路径查看
		//printf_Route(map_path, path_stack, top,3);  //迭代3次寻优
        printf("初始规划的路径点个数%d\n",top+1);
        int iflag = top;
        for (; iflag >=0; iflag--)
        {
            geometry_msgs::Point32 pa;
            pa.x = (path_stack[iflag]->s_y) * map_resolution + map_frame_originx;
            pa.y = (map_height-path_stack[iflag]->s_x) * map_resolution + map_frame_originy; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
            pa.z = 0;
            orginaStartPath.points.push_back(pa); //该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标
           //int size=aStartPath.points.size();
           //ROS_INFO("第%d个径点的x为%6.2f",iflag,pa.x);
           //ROS_INFO("第%d个径点的y为%6.2f",iflag,pa.y);
           //ROS_INFO("aStartPath的径点个数为%d",size);
        }
    
       iflag = 0;
       int Validnewstartindex=top;   //path_stack的最后一个元素为起始点
       for(;;)
       {
           for (; iflag < Validnewstartindex; iflag++) //从目标点往回遍历规划好的路径点，若当前路径点与起始点之间的连线没有障碍物，则返回true跳出循环
           {                          //将当前路径点之前的路径点截断,保留当前点到目标点的路径点  保留有效路径点
              if(isValidLine(ptr,path_stack[iflag]->s_x,path_stack[iflag]->s_y, path_stack[Validnewstartindex]->s_x, path_stack[Validnewstartindex]->s_y))
              {
                break;
              }
           }

           if(iflag!=Validnewstartindex)
           {
                ValidPath_printf(path_stack,Validnewstartindex,iflag);

                geometry_msgs::Point32 worldPose;
                worldPose.x = (path_stack[iflag]->s_y) * map_resolution + map_frame_originx;
                worldPose.y = (map_height-path_stack[iflag]->s_x) * map_resolution + map_frame_originy; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
                if(iflag==0)
                    worldPose.z =  m_navPara.desired.h; 
                else
                    worldPose.z = 0;                                                                
               ValidaStartPath.points.push_back(worldPose); //该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标
               printf("第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f \n",iflag,worldPose.x,worldPose.y,worldPose.z);
           }
           else
              break;
  
           if(iflag!=0)
           {
              Validnewstartindex=iflag;
              iflag=0;
            } 
            else 
               break; 
        }

       if(iflag==Validnewstartindex)
       {
          iflag=iflag-1;
          for(; iflag >= 0; iflag--)
          {
                ValidPath_printf(path_stack,Validnewstartindex,iflag);

                geometry_msgs::Point32 worldPose;
                worldPose.x = (path_stack[iflag]->s_y) * map_resolution + map_frame_originx;
                worldPose.y = (map_height-path_stack[iflag]->s_x) * map_resolution + map_frame_originy; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
                if(iflag==0)
                    worldPose.z =  m_navPara.desired.h; 
                else
                    worldPose.z = 0;                                                                
               ValidaStartPath.points.push_back(worldPose); //该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标
               printf("第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f \n",iflag,worldPose.x,worldPose.y,worldPose.z);
           }
       }
        //对上面的路径点再次优化，相邻两个关键路径点之间如果相隔的距离小于0.3m，即30cm，则将前一个路径点踢出路径容器，
        //即将该点后面的路径点依次向前移动，并删除最后一个容器元素
        {
            int size = ValidaStartPath.points.size();
            for(char i=0;i<size-1;i++)
            {
                //计算相邻两个路径点之间的直线距离
                float planpointdistance = (ValidaStartPath.points[i].x-ValidaStartPath.points[i+1].x)*
                                          (ValidaStartPath.points[i].x-ValidaStartPath.points[i+1].x) +
                                          (ValidaStartPath.points[i].y-ValidaStartPath.points[i+1].y)*
                                          (ValidaStartPath.points[i].y-ValidaStartPath.points[i+1].y);
                planpointdistance=sqrt(planpointdistance);
                if(planpointdistance<=0.2)        //如果相邻两点之间直线距离小于30cm,则进行优化  注意0.3这个阈值很关键
                {
                    for(char j=i;j<size-1;j++)
                    {
                        ValidaStartPath.points[j].x=ValidaStartPath.points[j+1].x;
                        ValidaStartPath.points[j].y=ValidaStartPath.points[j+1].y;
                        ValidaStartPath.points[j].z=ValidaStartPath.points[j+1].z;
                    }
                    ValidaStartPath.points.pop_back();    //踢出容器末尾的元素
                    size = ValidaStartPath.points.size(); 
                    i=i-1;
                }
            }
            
        }

        int size = ValidaStartPath.points.size();
        printf("优化后的路径点个数%d \n",size);
        for(int i=0;i<size;i++)
        {
            printf("第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f \n",i,
            ValidaStartPath.points[i].x,ValidaStartPath.points[i].y,ValidaStartPath.points[i].z);
        }

        getPath = true;
	}
	else
	{
        printf("么有找到路径\n");
        getPath = false;
	}
    
	puts("");

	//释放之前动态申请的链表节点
    for(int i=0;i<row;i++)
      free(map_maze[i]);
	free(map_maze);
	free(open_table);
	free(close_table);
	free(path_stack);
    
    open_node_count=0;   	
	close_node_count=0; 
    top = -1;
     
    //还原起始点与目标点的状态
    ptr[rows_current_x][cols_current_y]=init_current_state;
	ptr[rows_desired_x][cols_desired_y]=init_desired_state;

    return 0;
}

void ValidPath_printf(pAStarNode *path_stack,int Validnewstartindex,int iflag)
{
    float xDistance = path_stack[iflag]->s_x - path_stack[Validnewstartindex]->s_x;
    float yDistance = path_stack[iflag]->s_y - path_stack[Validnewstartindex]->s_y;
    float Distance = sqrt(xDistance * xDistance + yDistance * yDistance); 
    geometry_msgs::Point32 worldPose;
    int tempx, tempy;

    for (float i=1; i<=Distance; i++)
        {
            tempx = (int)round(path_stack[Validnewstartindex]->s_x + i * xDistance / Distance);
            tempy = (int)round(path_stack[Validnewstartindex]->s_y + i * yDistance / Distance); //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位  

            worldPose.x = tempy * map_resolution + map_frame_originx;
            worldPose.y = (map_height-tempx) * map_resolution + map_frame_originy; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
            worldPose.z = 0;      

            ValidgloabPath_printf.points.push_back(worldPose); //该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标
            printf("第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f \n",iflag,worldPose.x,worldPose.y,worldPose.z);
        }
}

bool isValidLine( uchar ** ptr, int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int ux = ((dx > 0) << 1) - 1; //x的增量方向，取或-1
    int uy = ((dy > 0) << 1) - 1; //y的增量方向，取或-1
    int x = x1, y = y1, eps;      //eps为累加误差
    
    eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy)
    {
        for (x = x1; x != x2; x += ux)
        {
            if (ptr[x][y] == BARRIER)
            {   
                return false;
            }
            eps += dy;
            if ((eps << 1) >= dx)
            {
                y += uy;
                eps -= dx;
            }
        }
    }
    else
    {
        for (y = y1; y != y2; y += uy)
        {
            if (ptr[x][y] == BARRIER)
            {
                return false;
            }
            eps += dx;
            if ((eps << 1) >= dy)
            {
                x += ux;
                eps -= dy;
            }
        }
    }
    return true;
}

bool isValidlocalLine(const std::vector<std::vector<char>> &maze, int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int ux = ((dx > 0) << 1) - 1; //x的增量方向，取或-1
    int uy = ((dy > 0) << 1) - 1; //y的增量方向，取或-1
    int x = x1, y = y1, eps;      //eps为累加误差

    eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy)
    {
        for (x = x1; x != x2; x += ux)
        {
            if (maze[x][y] == BARRIER)
            {
                return false;
            }
            eps += dy;
            if ((eps << 1) >= dx)
            {
                y += uy;
                eps -= dx;
            }
        }
    }
    else
    {
        for (y = y1; y != y2; y += uy)
        {
            if (maze[x][y] == BARRIER)
            {
                return false;
            }
            eps += dx;
            if ((eps << 1) >= dy)
            {
                x += ux;
                eps -= dy;
            }
        }
    }
    return true;
}

//用于判断某点8个方向范围内的点是否存在障碍物，返回true则表示没有障碍物，false则表示有障碍物
bool judgment_neighbors(int *rows_current_x, int *cols_current_y,int row,int col,int range)
{
    //向图像正下边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i<=(*rows_current_x+range);i++)
        for(int j=*cols_current_y;j<=*cols_current_y;j++)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像正上边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i>=(*rows_current_x-range);i--)
        for(int j=*cols_current_y;j<=*cols_current_y;j++)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像右边判断8个像素是否是障碍物
    for(int i= *rows_current_x;i<= *rows_current_x;i++)
        for(int j= *cols_current_y;j<=(*cols_current_y+range);j++)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像左边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i<=*rows_current_x;i++)
        for(int j=*cols_current_y;j>=(*cols_current_y-range);j--)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像右下角边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i<=(*rows_current_x+range);i++)
        for(int j=*cols_current_y;j<=(*cols_current_y+range);j++)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像左下角边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i<=(*rows_current_x+range);i++)
        for(int j=*cols_current_y;j>=(*cols_current_y-range);j--)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像右上角边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i>=(*rows_current_x-range);i--)
        for(int j=*cols_current_y;j<=(*cols_current_y+range);j++)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    //向图像左上角边判断8个像素是否是障碍物
    for(int i=*rows_current_x;i>=(*rows_current_x-range);i--)
        for(int j=*cols_current_y;j>=(*cols_current_y-range);j--)
        {
            if (i>= 0 && i<row && j >= 0 && j < col)
                if(ptr[i][j] != BARRIER)
                {
                    *rows_current_x=i;
                    *cols_current_y=j;
                    return true;
                }  
        }

    return false;
}

//局部路径规划,暂时未用到
int local_Astarplan(int row,int col,int map_count,float submap_resolution,HL::CarPose localdesired)
{
    orginlocalPath.points.clear();
    ValidlocalPath.points.clear();

    float submap_fx = localdesired.x - m_navPara.current.x;  //相对于map坐标系的距离值
    float submap_fy = localdesired.y - m_navPara.current.y;

    float boundx = (col - 1) * submap_resolution * 0.5;   //为什么要*0.5
    float boundy = (row - 1) * submap_resolution * 0.5;  //bound，子图边界
    float fk = (boundy / boundx);

    HL::CarPose tempGoal{submap_fx, submap_fy, localdesired.h};

    if ((fabs(submap_fx) > boundx) || (fabs(submap_fy) > boundy))  //将目标移动距离限制在子图范围内
    {
        localdesired.h=0;   //若当前目标点超出submap范围，将子图目标点航向角设定为０度；
        if (submap_fx == 0)
        {
            tempGoal.x = 0;
            tempGoal.y = ((submap_fy >= 0) ? boundy : -boundy);
        }
        else if (submap_fy == 0)
        {
            tempGoal.x = ((submap_fx >= 0) ? boundx : -boundx);
            tempGoal.y = 0;
        }
        else
        {
            float k = submap_fy / submap_fx;
            if (fabs(k) <= fk)
            {
                tempGoal.x = ((submap_fx >= 0) ? boundx : -boundx);
                tempGoal.y = k * tempGoal.x;     //根据公式得到另一个坐标,tan*x=y
            }
            else
            {
                tempGoal.y = ((submap_fy >= 0) ? boundy : -boundy);
                tempGoal.x = tempGoal.y / k;
            }
        }
    } //转换为局部子图中的局部目标，相对于子图中心坐标系
    int localend_y = floor(tempGoal.x / submap_resolution + 0.5) + col / 2;  //将实际距离转换为地图中的像素点位置
    int localend_x = floor(tempGoal.y / submap_resolution + 0.5) + row / 2; //floor向下取整，取不大于floor(x)的整数

    int localstart_x = row / 2;  //当前起始点
    int localstart_y = col / 2;
    
    ROS_INFO("current_x=%6.2f,current_y=%6.2f",m_navPara.current.x,m_navPara.current.y);
    ROS_INFO("localend_x=%6.2f,localend_y=%6.2f",tempGoal.x,tempGoal.y);
    ROS_INFO("localstart_x=%d,localstart_y=%d",localstart_x,localstart_y);
    ROS_INFO("localend_x=%d,localend_y=%d",localend_x,localend_y);

   // bool getPathflag = true;
    if (localstart_x == localend_x && localstart_y == localend_y)
    {
        printf("局部目标点与局部起始点重合,局部路径规划失败\n");
        return 1;
    }
    if (maze[localend_x][localend_y] == BARRIER)
    {
        printf("局部目标点是障碍物,局部路径规划失败\n");
        return 1;
    }
    printf("局部目标点设置正确,进行局部路径规划\n");

    int init_desired_state = maze[localend_x][localend_y];
    int init_current_state = maze[localstart_x][localstart_y];
	maze[localend_x][localend_y]=ENDNODE;
    maze[localstart_x][localstart_y]=STARTNODE;

    map_maze = (AStarNode  **)malloc(row * sizeof(AStarNode  *));  
	for (int k=0;k<row;k++)
		map_maze[k] = (AStarNode  *)malloc(col * sizeof(AStarNode));

	open_table  = (pAStarNode *)malloc(map_count * sizeof(pAStarNode));  
	close_table = (pAStarNode *)malloc(map_count * sizeof(pAStarNode)); 
	path_stack  = (pAStarNode *)malloc(map_count * sizeof(pAStarNode));  
	//path_stack_deal = (pAStarNode *)malloc(map_count * sizeof(pAStarNode));  

	AStarNode *start_node=NULL;		
	AStarNode *end_node= NULL;		
	AStarNode *curr_node= NULL;			

	int    is_found = 0;		 //找到路径标志位

	for (int i=0;i<row;++i)
	{
		for (int j=0; j<col;++j)
		{
			map_maze[i][j].s_g = 0;
			map_maze[i][j].s_h = 0;
			map_maze[i][j].s_is_in_closetable = 0;
			map_maze[i][j].s_is_in_opentable = 0;
			map_maze[i][j].s_style = maze[i][j];
			map_maze[i][j].s_x = i;
			map_maze[i][j].s_y = j;
			map_maze[i][j].s_parent = NULL;

			if (map_maze[i][j].s_style==STARTNODE)	// 起点
			{
				start_node=&(map_maze[i][j]);
			}
			else if (map_maze[i][j].s_style==ENDNODE)	// 终点
			{
				end_node=&(map_maze[i][j]);
			}
            else ;
			//printf("%d ", maze[i][j]);
		}
		//printf("\n");
	}

	open_table[open_node_count++] = start_node;			
	start_node->s_is_in_opentable = 1;			
	start_node->s_g = 0;
	start_node->s_h = abs(end_node->s_x - start_node->s_x) + abs(end_node->s_y - start_node->s_y);
	//start_node->s_h = (int)round(sqrt((end_node->s_x - start_node->s_x) *(end_node->s_x - start_node->s_x) + (end_node->s_y - start_node->s_y) * (end_node->s_y - start_node->s_y)));

	start_node->s_parent = NULL;
	if (start_node->s_x==end_node->s_x && start_node->s_y==end_node->s_y)
	{
        //释放之前动态申请的链表节点
        for(int i=0;i<row;i++)
          free(map_maze[i]);
	    free(map_maze);
	    free(open_table);
	    free(close_table);
	    free(path_stack);
    
        open_node_count=0;   	
	    close_node_count=0; 
        top = -1;
     
       //还原起始点与目标点的状态
        maze[localstart_x][localstart_y]=init_current_state;
	    maze[localend_x][localend_y]=init_desired_state;

        printf("起点==终点!\n");
		return 1;
	}

     while (1)
	{
		curr_node=open_table[0];		// open表的第一个点一定是f值最小的节点(通过堆排序得到的)
		open_table[0]=open_table[--open_node_count];	// 最后一个点放到open表的第一个点，然后进堆调整 此时已经在open表中删除当前节点 
		adjust_heap(0);				// 堆调整

		close_table[close_node_count++]=curr_node;	// 当前点加入到close表
		curr_node->s_is_in_closetable=1;		// 已经在close表中

		if (curr_node->s_x==end_node->s_x && curr_node->s_y==end_node->s_y)// 终点在close表，结束规划
		{
			is_found=1;
			break;
		}

		get_neighbors(curr_node, end_node,row,col);			// 对邻居的处理

		if (open_node_count==0)				// 没有找到路径点
		{
			is_found=0;
			break;
		}
	}

	//显示搜索到的最优路径
    top = -1;
	if (is_found)
	{
		curr_node=end_node;
		while (curr_node)      //top=0为目标点
		{
			path_stack[++top]=curr_node;
			curr_node=curr_node->s_parent;
		}
		//处理规划后的路径及在地图中输出路径查看
		//printf_Route(map_path, path_stack, top,3);  //迭代3次寻优
        printf("局部初始规划的路径点个数%d\n",top+1);
        int iflag = top;
        for (; iflag >=0; iflag--)
        {
            geometry_msgs::Point32 pa;
            pa.y = (path_stack[iflag]->s_x - localstart_x) * submap_resolution + m_navPara.current.y;
            pa.x = (path_stack[iflag]->s_y - localstart_y) * submap_resolution + m_navPara.current.x; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
            pa.z = 0;
            orginlocalPath.points.push_back(pa); //该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标
           //int size=aStartPath.points.size();
           //ROS_INFO("第%d个径点的x为%6.2f,第%d个径点的y为%6.2f",iflag,pa.x,pa.y);
           printf("局部第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f\n",iflag,pa.x,pa.y,pa.z);
           //ROS_INFO("aStartPath的径点个数为%d",size);
        }
        
       iflag = 0;
       int Validnewstartindex=top;   //path_stack的最后一个元素为起始点
       for(;;)
       {
           for (; iflag < Validnewstartindex; iflag++) //从目标点往回遍历规划好的路径点，若当前路径点与起始点之间的连线没有障碍物，则返回true跳出循环
           {                          //将当前路径点之前的路径点截断,保留当前点到目标点的路径点  保留有效路径点
              if(isValidlocalLine(maze,path_stack[iflag]->s_x,path_stack[iflag]->s_y, path_stack[Validnewstartindex]->s_x, path_stack[Validnewstartindex]->s_y))
              {
                break;
              }
           }

           if(iflag!=Validnewstartindex)
           {
              geometry_msgs::Point32 worldPose;
              worldPose.y = (path_stack[iflag]->s_x - localstart_x) * submap_resolution + m_navPara.current.y;
              worldPose.x = (path_stack[iflag]->s_y - localstart_y) * submap_resolution + m_navPara.current.x; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
              if(iflag==0)
                worldPose.z = localdesired.h; 
              else
                worldPose.z = 0;                                                                
              ValidlocalPath.points.push_back(worldPose); //该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标
              printf("局部第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f \n",iflag,worldPose.x,worldPose.y,worldPose.z);
           }
           else
              break;
  
           if(iflag!=0)
           {
              Validnewstartindex=iflag;
              iflag=0;
           } 
            else 
               break; 
        }

       if(iflag==Validnewstartindex)
       {
          iflag=iflag-1;
          for(; iflag >= 0; iflag--)
          {
            geometry_msgs::Point32 worldPose;
            worldPose.y = (path_stack[iflag]->s_x - localstart_x) * submap_resolution + m_navPara.current.y;
            worldPose.x = (path_stack[iflag]->s_y - localstart_y) * submap_resolution + m_navPara.current.x; //将找到的地图行列坐标转换为相对于map坐标系的实际位置 以m为单位
            if(iflag==0)
              worldPose.z = localdesired.h; 
            else
              worldPose.z = 0;                                                                 
            ValidlocalPath.points.push_back(worldPose);//该点云结构体存储的是规划好的相对于map坐标系的实际路径点坐标   此步将目标点放到队列的尾端
            printf("局部第%d个路径点:x为%6.2f,y为%6.2f,h为%6.2f \n",iflag,worldPose.x,worldPose.y,worldPose.z);
          }
       }

        int size = ValidlocalPath.points.size();
        printf("局部优化后的路径点个数%d \n",size);
        localgetPath = true;
	}
	else
	{
        printf("局部么有找到路径\n");
        localgetPath = false;
	}

	puts("");

	//释放之前动态申请的链表节点
    for(int i=0;i<row;i++)
      free(map_maze[i]);
	free(map_maze);
	free(open_table);
	free(close_table);
	free(path_stack);
    
    open_node_count=0;   	
	close_node_count=0; 
    top = -1;
     
    //还原起始点与目标点的状态
    maze[localstart_x][localstart_y]=init_current_state;
	maze[localend_x][localend_y]=init_desired_state;

    return 0;
}