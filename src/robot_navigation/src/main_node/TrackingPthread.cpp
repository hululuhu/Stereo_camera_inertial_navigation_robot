#include "TrackingPthread.h"
#include "../common/common.h"
#include "nav.h"

namespace HL
{

    static NavPara m_navPara = {{0, 0, 0}, {0, 0, 0}, false, false};    //导航参数 当前位姿，全局目标点位姿，新的目标标志位，开始导航标志位
    static std::vector<HL::CarPose> Gloab_path_points;  //路径规划节点发布的 路径点
    static float poseChangePow2=0;     //位置改变的范围
    static int reachFinalCount = 0;    //到达全局目标点计数   
    static bool hasNewPose = false;    //是否有新的定位信息 
    static bool getlocalDest=true;    //判断是否到达局部目标点
    static bool isFinalDest=false;    //判断下一个局部目标点是否全局目标点
    static bool reachFinal = false;   //判断是否到达最后目标点
    static bool motion_mode = false;  //false为手柄遥控模式，true为导航模式
    static uint32_t  Gloab_path_points_index=1;  

    inline void limit(float &v, float left, float right)
    {
        assert(left < right);

        if (v > right)
            v = right;
        else if (v < left)
            v = left;
    }

    TrackingPthread::TrackingPthread()
    {
        pthread_create(&id, NULL, MyPthread, (void *)this);
    }

    TrackingPthread::~TrackingPthread()
    {
        pthread_detach(id);
    }

    void *TrackingPthread::MyPthread(void *temp)
    {
        TrackingPthread *t = (TrackingPthread *)temp;
        t->DoPthread();
    }

    void *TrackingPthread::DoPthread(void)
    {
        ros::NodeHandle n;

        {
            ros::NodeHandle nh("~");

            if (!nh.getParam("maxForwardSpeed", maxForwardSpeed))
                maxForwardSpeed = 0.2;
            if (!nh.getParam("maxBackSpeed", maxBackSpeed))
                maxBackSpeed = -0.2;
            if (!nh.getParam("maxOmega", maxOmega))
                maxOmega = 1.0;
            if (!nh.getParam("maxUpAcc", maxUpAcc))
                maxUpAcc = 0.5;
            if (!nh.getParam("maxBackAcc", maxBackAcc))
                maxBackAcc = -1;
            if (!nh.getParam("poseChange",  poseChange))
                poseChange = 0.1;
            poseChangePow2 = poseChange*poseChange;

            //距离容差与航向角容差
            if (!nh.getParam("maxDisErr", maxDisErr))
                maxDisErr = 0.05;
            //转向角度容差
            if (!nh.getParam("fabsdfh", fabsdfh))
                fabsdfh = 0.2;    
            //航向角度容差    
            if (!nh.getParam("maxAngErr", maxAngErr))
                maxAngErr = 0.05;

            //位置PID参数
            if (!nh.getParam("DisPID_P",  PID[0]))
                PID[0] = 20.0;
            if (!nh.getParam("DisPID_I",  PID[1]))
                PID[0] = 5.0;
            if (!nh.getParam("DisPID_D",  PID[2]))
                PID[0] = 0.0;

            //航向角PID参数
            if (!nh.getParam("AngPID_P",  PID[3]))
                PID[0] = 80.0;
            if (!nh.getParam("AngPID_I",  PID[4]))
                PID[0] = 5.0;
            if (!nh.getParam("AngPID_D",  PID[5]))
                PID[0] = 5.0;

            //导航控制参数
            if (!nh.getParam("newGoal",  newGoal))
                newGoal = false;
            if (!nh.getParam("startNav",  startNav))
                startNav = false;
            if (!nh.getParam("emergeStop",  emergeStop))
                emergeStop = false;
        }
        
        ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &TrackingPthread::joyCallback, this);
        //ros::Subscriber key_sub = n.subscribe("cmd_vel", 2, &TrackingPthread::cmd_keyCallback, this);
        ros::Subscriber orbslam2_pose_sub = n.subscribe<nav_msgs::Odometry>("/orb_slam/world_bf_pose", 2, boost::bind(&TrackingPthread::orb_pose_input, this,_1));  //订阅orbslam2定位位姿
        ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, boost::bind(&TrackingPthread::PoseReceived, this,_1));
        ros::Subscriber gloab_path_sub = n.subscribe("gloab_Validpath", 2, &TrackingPthread::path_Callback, this);    //订阅全局路径点导航
        ros::Subscriber stopcar_sub = n.subscribe<std_msgs::Int32>("main_navigation/Stopcar", 10, boost::bind(&TrackingPthread::Stopcarcallback,this,_1));

        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 2);

        //ros::Timer timer=n.createTimer(ros::Duration(0.02),timerCallback,false);
        ros::Rate loop_rate(50);    //20ms
        while (ros::ok())
        {
            ros::spinOnce();
            if(motion_mode)     //运动模式处于自动导航模式
            {
                getNavcmd();
                vel_pub.publish(vel);
            }

            loop_rate.sleep();
        }
    }

    void TrackingPthread::cmd_keyCallback(const geometry_msgs::Twist::ConstPtr &cmd)
    {
        maxDisErr = 0.05;
        maxAngErr = 0.05;
        fabsdfh   = 0.3;
        
        //读取手柄发送的PID参数
        PID[0] = 20;
        PID[1] = 5;
        PID[2] = 0;
        PID[3] = 80;
        PID[4] = 5;
        PID[5] = 5;

        startNav = false;
        emergeStop = false;
        newGoal = false;
    }

    void TrackingPthread::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        if(joy->buttons[10] == 1)  //运动模式标志位  手柄SELECT按键
        {
            motion_mode = true;   //true为导航模式
            ROS_INFO("motion_mode is navigation mode!");
        }
        if(joy->buttons[11] == 1)  //运动模式标志位  手柄START按键
        {
            motion_mode = false;   //false为遥控模式，解锁手柄其他按键以用来遥控
            startNav = false;
            emergeStop =false;
            newGoal = false;
            ROS_INFO("motion_mode is Teleop_joy mode!");
        }
        if(motion_mode)
        {
            //设置距离容差与、转向容差、航向角容差
            if(joy->axes[7] == 1 || joy->axes[7] == -1)    //手柄 上下+/-
            {
                maxDisErr += 0.05 * joy->axes[7];
                ROS_INFO("maxDisErr = %f",maxDisErr);
            }
            if(joy->axes[6] == 1 || joy->axes[6] == -1)    //手柄 左右+/-
            {
                maxAngErr += 0.05 * joy->axes[6];
                ROS_INFO("maxAngErr = %f",maxAngErr);
            }
            if(joy->buttons[3] == 1 )           //手柄 X
            {
                fabsdfh += 0.05 *joy->buttons[3];
                ROS_INFO("fabsdfh = %f",fabsdfh);
            }
            if(joy->buttons[0] == 1 )           //手柄 A
            {
                fabsdfh += -0.05 *joy->buttons[0];
                ROS_INFO("fabsdfh = %f",fabsdfh);
            }

            //导航控制信息
            if(joy->buttons[4] == 1)    //手柄Y按键 开始导航
                startNav = true;
            if(joy->buttons[6] == 1 || joy->buttons[7] == 1) //手柄L1或R1按键 急停
            {
                startNav = false;
                emergeStop =true;
                newGoal = false;
            }
        }
    }

    void TrackingPthread::PoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose)
    {
        m_navPara.current.x = (float)pose->pose.pose.position.x; 
        m_navPara.current.y = (float)pose->pose.pose.position.y;  
        m_navPara.current.h = (float)(tf::getYaw(pose->pose.pose.orientation));
        
        hasNewPose = true;
    }

    void TrackingPthread::orb_pose_input(const nav_msgs::OdometryConstPtr &pose)
    {
        m_navPara.current.x = (float)pose->pose.pose.position.x;
        m_navPara.current.y = (float)pose->pose.pose.position.y;
        m_navPara.current.h = (float)(tf::getYaw(pose->pose.pose.orientation));

        hasNewPose = true;
        //ROS_INFO("订阅到ORBSLAM2 pose!");
    }

    void TrackingPthread::path_Callback(const sensor_msgs::PointCloudConstPtr &msgpoints)
    {     
        Gloab_path_points.clear();
        
        getlocalDest=true;
        isFinalDest=false;
        reachFinalCount = 0;
        reachFinal = false;   
        Gloab_path_points_index=1;

        int pointsize=msgpoints->points.size();
        ROS_INFO("优化后的路径点个数%d",pointsize);

        for(int iflag=0; iflag < pointsize; iflag++)
        { 
            CarPose worldPose;
            worldPose.x = msgpoints->points[iflag].x;
            worldPose.y = msgpoints->points[iflag].y; //将当前路径点相对于子图中心的距离转换为相对于map坐标的实际距离
            worldPose.h = msgpoints->points[iflag].z; 
            Gloab_path_points.push_back(worldPose); //该点云结构体中存储的是规划好的相对于world坐标系的路径点坐标
            ROS_INFO("第%d路径点的:x为%6.2f,y为%6.2f,h为%6.2f",iflag,worldPose.x,worldPose.y,worldPose.h);
        }

        m_navPara.desired.x = msgpoints->points[pointsize-1].x;
        m_navPara.desired.y = msgpoints->points[pointsize-1].y;
        m_navPara.desired.h = msgpoints->points[pointsize-1].z;
        newGoal = true; //订阅到新路径，说明有新的目标点被设置
        
        int Gloabpointsize=Gloab_path_points.size();
        ROS_INFO("订阅到的全局路径点个数%d",Gloabpointsize);
        ROS_INFO("Gloab_path_points容器已订阅到优化后的路径点消息");
    }

    void TrackingPthread::Stopcarcallback(const std_msgs::Int32::ConstPtr msg)
    {
        if(msg->data==2)
        {
            // 接受到停车指令, 则停车
            vel.linear.x = 0;
            vel.angular.z = 0;
            ROS_INFO("meet occd Stopcar again plan path!");
       
            startNav = false;
            emergeStop = false;
            newGoal = false;

            Gloab_path_points.clear();
        }
        else if(msg->data==1)
        {
            getlocalDest=true;
            isFinalDest=false;
            reachFinalCount = 0;
            reachFinal = false;

            startNav = true;
            emergeStop = false;
            newGoal = true;

            ROS_INFO("start car。。。。");
        }
        else ;
    }


    void TrackingPthread::getNavcmd(void)
    {
        m_navPara.startNav = startNav;
        m_navPara.emergeStop = emergeStop;
        m_navPara.newGoal = newGoal;
    
        if (m_navPara.emergeStop)
        {
            // j接收到急停指令, 则停车
            vel.linear.x = 0;
            vel.angular.z = 0;
            ROS_INFO("Emerge Stop");

            startNav = false;
            emergeStop = false;
            newGoal = false;
            
            Gloab_path_points.clear();
        }

        if (m_navPara.startNav)
        {
            static int timeout = 0;
            //ROS_INFO("startNav。。。。");
            if (m_navPara.newGoal || hasNewPose)
            {
                if (m_navPara.newGoal)   //有新的全局目标点
                {
                    reachFinalCount = 0;
                    reachFinal = false;
                    isFinalDest=false;
                    ROS_INFO("New destnation!");
                }
                
                CalNavCmdVel(&m_navPara, vel); // 仅当有新目标或者有新位置才重新计算控制指令          
    
                m_navPara.newGoal = false;
                newGoal = false;
                timeout = 0;
            //hasNewPose=false;
            }
            else
            {
                // 判定超时
                timeout++;
                if (timeout >= 20)
                {
                    // 20次循环都没有新数据则超时,且速度不为零(速度为零说明已到达目标点)
                    timeout =0;
                    if((int)(vel.linear.x*1000)!=0 && (int)(vel.angular.z*1000) !=0)   //说明定位模块已经挂掉
                    {
                        vel.linear.x = 0;
                        vel.angular.z = 0;
                        ROS_ERROR("Position not updated, timeout!");
                    }
                }
            }
        }
        else
        {
            // 停止导航时, 停车
            vel.linear.x = 0;
            vel.angular.z = 0;
        }
        //  ROS_INFO("%d    %d",m_navPara.startNav ,m_navPara.emergeStop );
    }

    // 轨迹跟踪, 计算控制指令, 速度, 角速度 
    // 输入: const NavPara * 导航指令
    //       geometry_msgs::Twist & 控制指令
    void TrackingPthread::CalNavCmdVel(const NavPara *nav, geometry_msgs::Twist &ctr)
    {
        static uint8_t pricnt = 0;
        const uint8_t PCT = 5;
        pricnt++;

        // 计算本次期望位姿
        static CarPose nextDest={0,0,0};
        //static uint32_t  Gloab_path_points_index=1;
        if(getlocalDest && isFinalDest==false)    //如果已经到达局部目标点，且未到达全局目标点
        {
        if (Gloab_path_points_index<Gloab_path_points.size())
        {
            // 若存在期望路径, 则设定期望位姿为下一个路径点
            nextDest.x = Gloab_path_points[Gloab_path_points_index-1].x;
            nextDest.y = Gloab_path_points[Gloab_path_points_index-1].y;
            nextDest.h = Gloab_path_points[Gloab_path_points_index-1].h;
            isFinalDest = false;
            getlocalDest = false;
            Gloab_path_points_index++;       

        //if (pricnt % (3 * PCT) == 1)
                ROS_INFO("nextDest=[%6.3f,%6.3f,%6.3f]", nextDest.x, nextDest.y,nextDest.h);
        }else{
            // 否则, 直接将最终目标作为期望位姿

            // 此处需要加防碰撞处理，即A*算法没有求出路径
            Gloab_path_points_index=1;
            nextDest = nav->desired;
            getlocalDest = false;
            isFinalDest = true;
            ROS_INFO("FinalDest=[%6.3f,%6.3f,%6.3f]", nextDest.x, nextDest.y,nextDest.h);
            Gloab_path_points.clear();
        }
    }

        // 求位置闭环控制量, 期望速度, 期望角速度
        float dx, dy, dph, dfh, ds;
        dx = nextDest.x - nav->current.x;
        dy = nextDest.y - nav->current.y;
        ds = sqrt(dx * dx + dy * dy);

        dph = angle_diff(nextDest.h, nav->current.h);   //目标点期望航向角度与当前航向角之间的误差
        dfh = angle_diff(atan2(dy, dx), nav->current.h); //转向与当前航向角之间的误差

        if (fabs(dfh) > M_PI / 2)
        {
            if (dfh > M_PI / 2)
                dfh = dfh - M_PI;
            else
                dfh = dfh + M_PI;
            ds = -ds;
        }

        assert(dfh <= M_PI && dfh >= -M_PI);

        static float last_ds = 0, ids = 0;
        static float last_vx = 0, last_az = 0;
        float dds = ds - last_ds;

        ids += ds;
        last_ds = ds;
        limit(ids, -2, 2);
        limit(dds, -0.5, 0.5);
        float vx = (ds * PID[0] + ids * PID[1] + dds * PID[2]) / 100.0;    //位置pid闭环
        limit(vx, maxBackSpeed, maxForwardSpeed);

        static float ldfh = 0, idfh = 0;
        float ddfh = dfh - ldfh;
        idfh += dfh;
        ldfh = dfh;
        limit(idfh, -2, 2);
        limit(ddfh, -1, 1);
        float afz = (dfh * PID[3] + idfh * PID[4] + ddfh * PID[5]) / 100.0;  // 转向pid闭环
        limit(afz, -maxOmega, maxOmega);

        static float ldph = 0, idph = 0;
        float ddph = dph - ldph;
        idph += dph;
        ldph = dph;
        limit(idph, -2, 2);
        limit(ddph, -1, 1);
        float apz = (dph * PID[3] + idph * PID[4] + ddph * PID[5]) / 100.0;  //目标点航向角pid闭环
        limit(apz, -maxOmega, maxOmega);

        // 导航逻辑
        float az = 0;    

        if (!reachFinal)    //未到达最后的位置
        {
            // 未停在目标位置
            if (fabs(ds) > maxDisErr)
            {
                // 若未到达期望转向角度
                if (fabs(dfh) > fabsdfh)
                {
                    // 若航向错误, 则转向
                    idph = ldph = 0;
                    ids = last_ds = 0;
                    vx = 0;
                    az = afz;
                }else{
                    // 航向正确, 则行进
                    idph = ldph = 0;
                    az = afz;
                }
                reachFinalCount = 0;
            }
            else
            {
                // 若到达最终期望位置, 等待平稳停车
                if(isFinalDest)
                {
                    reachFinalCount++;
                    if (reachFinalCount > 3)
                    {
                        reachFinal = true;
                        ROS_INFO("Reached Final position![%6.3f,%6.3f,%6.3f]", nextDest.x, nextDest.y,nextDest.h);
                        ROS_INFO("Final position err is: !\n"
                                "dis_err=%6.3f/%6.3f ang_err=%6.3f/%6.3f",
                                ds, maxDisErr, dph, maxAngErr);
                    }
                    vx = 0;
                    az = 0;
                }
            getlocalDest=true;
            ROS_INFO("到达目标点[%6.3f,%6.3f]", nextDest.x, nextDest.y);
            }
        }
        else
        {
            // 已停止在最终目标位置,控制达到期望航向角
            if (fabs(dph) > maxAngErr)
            {
                // 若未到达期望角度, 则转向
                idfh = ldfh = 0;
                ids = last_ds = 0;
                vx = 0;
                az = apz;
            }else{
                // 到达期望角度
                vx = az = 0;
                ctr.linear.x = 0;
                ctr.angular.z = 0;
                last_ds = ids = 0;
                ldfh = idfh = 0;
                ldph = idph = 0;
                last_vx = last_az = 0;
                /*if (pricnt % (3 * PCT) == 1)
                ROS_INFO("Reached angle!\n"
                        "dis_err=%6.3f/%6.3f ang_err=%6.3f/%6.3f",
                        ds, maxDisErr, dph, maxAngErr);*/
                //到达最终目标点初始化所有变量为下次导航做准备
            // initallnavstate();
            
            }
        }

        // 限制最大加速度
        float vdv = vx - last_vx;
        if (vdv > 0)
        {
            if (vdv > (maxUpAcc * 0.02))
                vx = last_vx + maxUpAcc * 0.02;
        }
        else if (vdv < 0)
        {
            if (vdv < (maxBackAcc)*0.02)
                vx = last_vx + maxBackAcc * 0.02;
        }

        limit(vx, maxBackSpeed, maxForwardSpeed);
        limit(az, -maxOmega, maxOmega);

        last_vx = vx;
        last_az = az;

        ctr.linear.x = vx;
        ctr.angular.z = az;
    //   ROS_INFO("%f,%f",vx,az);
    }

    void TrackingPthread::initallnavstate(void)
    {
        Gloab_path_points.clear();
        
        getlocalDest=true;
        isFinalDest=false;
        reachFinalCount = 0;
        reachFinal = false;

        m_navPara.startNav=false;
        m_navPara.emergeStop=false;
        m_navPara.newGoal = false;

        startNav = false;
        emergeStop = false;
        newGoal = false;
    }
}