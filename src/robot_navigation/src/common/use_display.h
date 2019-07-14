#ifndef USE_DISPLAY_H
#define USE_DISPLAY_H

#include <QStringList>
#include "qbuttongroup.h"
#define  CAMBER  0.0174533  // 1°对应的弧度
#define  PI 3.14159265
#define  Du_Hudu(x) ((x)*CAMBER)
#define  DIA 2500
#define  TAG 720
#define  PIXMAP_X     850
#define  PIXMAP_Y     800
#define  Ultra_Num    8

double distance[TAG];
int tem_x,tem_y;
double temp_x,temp_y;
QStringList rowLabels;
QButtonGroup getModeSelect;
//the distance of obstacle in four directions of the robot
#define     dis_forward            (distance[718]+distance[719]+distance[0]+distance[1]+distance[2])/5.0
#define     dis_right              (distance[177]+distance[178]+distance[179]+distance[180]+distance[181])/5.0
#define     dis_back               (distance[357]+distance[358]+distance[359]+distance[360]+distance[361])/5.0
#define     dis_left               (distance[537]+distance[538]+distance[539]+distance[540]+distance[541])/5.0
//the width of obstacle in four directions of the robot
#define     wid_forward            sqrt(distance[89]*distance[89] + distance[629]*distance[629])
#define     wid_right              sqrt(distance[89]*distance[89] + distance[269]*distance[269])
#define     wid_back               sqrt(distance[269]*distance[269] + distance[439]*distance[439])
#define     wid_left               sqrt(distance[439]*distance[439] + distance[629]*distance[629])
//select the distance of the laser displayed
#define    Show_All     1       //origin
#define    Show_2M      2      //less 2m
#define    Show_1M      3      //less 1m
#define    Set_First_Limit      2000
#define    Set_Second_Limit     1000

#define    Safe_Distance     500.0   //mm
#define    Avoid_Distance    1000.0   //mm

#define    Grid_Size       10   //  10cm per grid
#define    Grid_COE        5
#define    ONE_GRID     Grid_COE*Grid_Size

#define    Car_R           1.575*ONE_GRID
#define    Laser_R         0.435*ONE_GRID

#define    Car_Central_X     5.425*ONE_GRID + Car_R
#define    Car_Central_Y     10.425*ONE_GRID+Car_R

#define    ONE_Ultra_X       Car_Central_X + sqrt(Car_R*Car_R-0.44*ONE_GRID*0.44*ONE_GRID)
#define    ONE_Ultra_Y       Car_Central_Y + 0.44*ONE_GRID

#define    TWO_Ultra_X          Car_Central_X + sqrt(Car_R*Car_R-0.44*ONE_GRID*0.44*ONE_GRID)
#define    TWO_Ultra_Y          Car_Central_Y - 0.44*ONE_GRID

#define    THREE_Ultra_X       Car_Central_X + 0.95*ONE_GRID
#define    THREE_Ultra_Y       Car_Central_Y - sqrt(Car_R*Car_R-0.95*ONE_GRID*0.95*ONE_GRID)

#define FOUR_Ultra_X         Car_Central_X + 0.5*ONE_GRID
#define FOUR_Ultra_Y         Car_Central_Y - sqrt(Car_R*Car_R-0.5*ONE_GRID*0.5*ONE_GRID)

#define  FIVE_Ultra_X          Car_Central_X - 0.5*ONE_GRID
#define  FIVE_Ultra_Y          Car_Central_Y - sqrt(Car_R*Car_R-0.5*ONE_GRID*0.5*ONE_GRID)

#define  SIX_Ultra_X           Car_Central_X - 0.95*ONE_GRID
#define  SIX_Ultra_Y           Car_Central_Y - sqrt(Car_R*Car_R-0.95*ONE_GRID*0.95*ONE_GRID)

#define  SEVEN_Ultra_X     Car_Central_X - sqrt(Car_R*Car_R-0.44*ONE_GRID*0.44*ONE_GRID)
#define  SEVEN_Ultra_Y     Car_Central_Y - 0.44*ONE_GRID

#define  EIGHT_Ultra_X     Car_Central_X - sqrt(Car_R*Car_R-0.44*ONE_GRID*0.44*ONE_GRID)
#define  EIGHT_Ultra_Y     Car_Central_Y + 0.44*ONE_GRID

float  Ultra1_Central_x=0,Ultra1_Central_y=0;
float  Ultra2_Central_x=0,Ultra2_Central_y=0;
float  Ultra3_Central_x=0,Ultra3_Central_y=0;
float  Ultra4_Central_x=0,Ultra4_Central_y=0;
float  Ultra5_Central_x=0,Ultra5_Central_y=0;
float  Ultra6_Central_x=0,Ultra6_Central_y=0;
float  Ultra7_Central_x=0,Ultra7_Central_y=0;
float  Ultra8_Central_x=0,Ultra8_Central_y=0;

float dis[8] = {0};  //the distance of ultrasonic
#endif // USE_DISPLAY_H
