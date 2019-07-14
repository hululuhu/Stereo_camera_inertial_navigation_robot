#ifndef PARAS_H
#define PARAS_H

#include "common.h"

namespace HL {

#pragma pack(4)
typedef struct
{
    volatile uint16_t addr ;
    volatile int32_t val ;
}ElementTable;

typedef enum
{
    CONTROL =0x0000,   // v , w ,x ,y ,theta   //float
    MSG_CONTROL =0x0020, //show msg  v,w,x,y,theta   //float，实际测到的线速度与角速度
    MSG_IMU = 0x0040,  //show msg  acc_x,y,z  gyr_x_y_z   //int16_t
    MSG_Ultrasonic = 0x0060,  //超声  int
    BTN_SWITCH =0x0080,   //状态开关，键盘控制(bit 0),IMU 初始化(bit 1), 开始导航(bit 2), 急停(bit 3) , newGoal (biy4)
    ADD_PID = 0x1000,   //pid 参数
    ADD_ERR = 0x1200,   //err
    NULL2  =0x2000
}ParaAddress;

typedef enum
{
     R_REGISTER =0x5f,
     W_REGISTER =0xf5,
     NULL1
}FunctionCode;

typedef struct
{
    FunctionCode fuc;
    uint16_t len;   //int32_t unit
    ParaAddress addr ;
    int32_t* data ;
}ParaGetSet;





#define KEY_VEL_CTR     0x00000001
#define KEY_INIT_IMU    0x00000002
#define KEY_START_NAV   0x00000004
#define KEY_EME_STOP    0x00000008
#define KEY_NEW_GOAL    0x00000010

#define Gyro_Gr	  (2000.0/65536.0*3.1415926/180.0)//(0.0005326/2.0)
#define Acc_Mss   ((4.0*9.81)/65536.0)


class Paras
{
public:
    Paras();
    ~Paras();
    bool SetAddressValue(const ParaGetSet &para);
    bool GetAddressValue(ParaGetSet &para);
    static void get_distance(double *dis);
    static void set_distance(double *dis);
    void ResetKeyRegister(void);
};

}

#endif // PARAS_H
