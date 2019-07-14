#include "paras.h"
#include <qdebug.h>
#include  "../server/uartlaser.h"


namespace HL {

static uint16_t dataTableLen=0;
static pthread_mutex_t g_tMutex  = PTHREAD_MUTEX_INITIALIZER;
static double send_distance[PACKLEN] = {0};
static pthread_mutex_t g_tLaserMutex  = PTHREAD_MUTEX_INITIALIZER;

static volatile zw::ElementTable dataTable[]=
{
    {CONTROL,0},  //v, w ,x ,y ,theta
    {CONTROL+1,0},
    {CONTROL+2,0},
    {CONTROL+3,0},
    {CONTROL+4,0},

    {MSG_CONTROL,0},
    {MSG_CONTROL+1,0},
    {MSG_CONTROL+2,0},
    {MSG_CONTROL+3,0},
    {MSG_CONTROL+4,0},

    {MSG_IMU,0},
    {MSG_IMU+1,0},
    {MSG_IMU+2,0},
    {MSG_IMU+3,0},
    {MSG_IMU+4,0},
    {MSG_IMU+5,0},

    {MSG_Ultrasonic,0},
    {MSG_Ultrasonic+1,0},
    {MSG_Ultrasonic+2,0},
    {MSG_Ultrasonic+3,0},
    {MSG_Ultrasonic+4,0},
    {MSG_Ultrasonic+5,0},
    {MSG_Ultrasonic+6,0},
    {MSG_Ultrasonic+7,0},

    {BTN_SWITCH,0},

    {ADD_PID, 	    500},  //PID1    //速度PID   X100
    {ADD_PID+1,		15},
    {ADD_PID+2,     0},
    {ADD_PID+3,  	50},   //PID2   //转角PID	X100
    {ADD_PID+4,   	10},
    {ADD_PID+5,   	0},
    {ADD_PID+6,   	20},    //PID3  //pose     X100
    {ADD_PID+7,  	5},
    {ADD_PID+8,   	0},
    {ADD_PID+9,   	80},    //PID4  //heading   X100
    {ADD_PID+10,  	5},
    {ADD_PID+11,   	5},

    {ADD_ERR,    50},     //pose err   X1000
    {ADD_ERR+1,  50},     //heading err  X1000
};


Paras::Paras()
{
    static uint8_t p_cnt=0;
    if(p_cnt==0){
    dataTableLen =  (sizeof(dataTable))/(sizeof(uint32_t)+sizeof(uint32_t));
    p_cnt++;
    }
}

Paras::~Paras()
{

}

bool Paras::SetAddressValue(const ParaGetSet &para)
{
    if(para.len>0)
    {
        uint16_t i,j;
        for(i=0;i<dataTableLen;i++)
        {
             if(dataTable[i].addr == para.addr)
             {
                uint16_t end=i+para.len;
                if((end<=dataTableLen)&&
                   (dataTable[end-1].addr == (para.addr+para.len-1))){
                    pthread_mutex_lock(&g_tMutex);
                    for(j=i;j<end;j++)
                        dataTable[j].val=para.data[j-i];
                    pthread_mutex_unlock(&g_tMutex);
                    return true;
                }else
                    return false;
             }
        }
    }
    return false;
}

bool Paras::GetAddressValue(ParaGetSet &para)
{
    if(para.len>0)
    {
        uint16_t i,j;
        for(i=0;i<dataTableLen;i++)
        {
             if(dataTable[i].addr == para.addr)
             {
                uint16_t end=i+para.len;
                if((end<=dataTableLen)&&
                   (dataTable[end-1].addr == (para.addr+para.len-1))){
                    pthread_mutex_lock(&g_tMutex);
                    for(j=i;j<end;j++)
                        para.data[j-i]=dataTable[j].val;
                    pthread_mutex_unlock(&g_tMutex);
                    return true;
                }else
                    return false;
             }
        }
    }
    return false;
}

void Paras::get_distance(double dis[])
{
      pthread_mutex_lock(&g_tLaserMutex );
      for(int i=0;i<PACKLEN;i++)
      {
          dis[i] = send_distance[i];
      }
      pthread_mutex_unlock(&g_tLaserMutex );
}

void Paras::set_distance( double dis[])
{
    pthread_mutex_lock(&g_tLaserMutex );
    for(int i=0;i<PACKLEN;i++)
    {
      send_distance[i]= dis[i];
    }
    pthread_mutex_unlock(&g_tLaserMutex );
}

void Paras::ResetKeyRegister(void)
{
    int32_t dat[1];
    zw::ParaGetSet packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
    zw::Paras m_para;
    m_para.GetAddressValue(packInfo);
    dat[0]&=(~KEY_INIT_IMU);
    packInfo.fuc =zw::W_REGISTER;
    m_para.SetAddressValue(packInfo);
}

}


