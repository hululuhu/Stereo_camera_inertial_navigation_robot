#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
static bool motion_mode = false;  //false为手柄遥控模式，true为导航模式
    
class TeleopTurtle
{
  public:
    TeleopTurtle();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_lx,linear_rx,linear_y,angular_lz,angular_rz;
    double l_scale_lx,l_scale_rx,l_scale_y,a_scale_lz,a_scale_rz;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

TeleopTurtle::TeleopTurtle():
  linear_lx(1),
  linear_rx(3),
  linear_y(4),
  angular_lz(0),
  angular_rz(2)
{
  nh_.param("axis_linear_lx", linear_lx, linear_lx);
  nh_.param("axis_linear_rx", linear_rx, linear_rx);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("axis_angular_lz", angular_lz, angular_lz);
  nh_.param("axis_angular_rz", angular_rz, angular_rz);
  nh_.param("scale_angular_lz", a_scale_lz, a_scale_lz);
  nh_.param("scale_angular_rz", a_scale_rz, a_scale_rz);
  nh_.param("scale_linear_lx", l_scale_lx, l_scale_lx);
  nh_.param("scale_linear_rx", l_scale_rx, l_scale_rx);
  nh_.param("scale_linear_y", l_scale_y, l_scale_y);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->buttons[10] == 1)  //运动模式标志位  手柄SELECT按键
  {
      motion_mode = true;   //true为导航模式
      ROS_INFO("motion_mode is navigation mode!");
  }
  if(joy->buttons[11] == 1)  //运动模式标志位  手柄START按键
  {
      motion_mode = false;   //false为遥控模式，解锁手柄其他按键以用来遥控
      ROS_INFO("motion_mode is Teleop_joy mode!");
  }
  if(!motion_mode)
  {
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_lz*joy->axes[angular_lz];
    twist.angular.z += a_scale_rz*joy->axes[angular_rz];
    twist.linear.x = l_scale_lx*joy->axes[linear_lx];
    twist.linear.x += l_scale_rx*joy->axes[linear_rx];

    //twist.linear.y = l_scale_y*joy->axes[linear_y];  //全向移动开启
    vel_pub_.publish(twist);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
