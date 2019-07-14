#include<iostream>
#include<assert.h>
#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>
#include<octomap/octomap.h>

using namespace std;  

int main (int argc, char **argv)  
{  
    ros::init (argc, argv, "pcd_pub");  
    
    if (argc != 3)
    {
        cout<<"Usage: pcd2octomap <input_pcdfile> <output_octomapfile>"<<endl;
        return -1;
    }

    ros::NodeHandle nh;  
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcd_topic_pub", 10);  
    
    string input_file = argv[1], output_file = argv[2];
    printf("input_file: %s\n", argv[1]);
    printf("output_file: %s\n", argv[2]);

    pcl::PointCloud<pcl::PointXYZI> cloud;    
    pcl::io::loadPCDFile (input_file, cloud);  
    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.05 );
 
    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
 
    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary( output_file );
    cout<<"\nconvert done."<<endl;

    bool ispub_ponitcloud;
    nh.getParam("ispub_pointcloud", ispub_ponitcloud);
    if(ispub_ponitcloud)
    {
        sensor_msgs::PointCloud2 output; 
        pcl::toROSMsg(cloud,output);// 转换成ROS下的数据类型 最终通过topic发布
        output.header.frame_id  ="world";

        ros::Rate loop_rate(1);  
        while (ros::ok())  
        {  
            output.header.stamp=ros::Time::now();
            pcl_pub.publish(output);  
            ros::spinOnce();  
            loop_rate.sleep();  
        }  
    }
    return 0;  
}