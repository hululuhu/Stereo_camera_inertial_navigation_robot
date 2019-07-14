#ifndef COMMON_H
#define COMMON_H

#include <string>


namespace HL {


typedef unsigned char uint8_t;
typedef char int8_t;
typedef unsigned short uint16_t;
typedef unsigned short uint16;
typedef short int16_t;
typedef unsigned int uint32_t;
typedef int int32_t;
typedef uint8_t byte;




typedef union
{
    float f;
    byte b[4];
}Hex2Float;

typedef union
{
    int32_t i;
    byte b[4];
}Hex2Int32;

typedef union
{
    int16_t i;
    byte b[2];
}Hex2Int16;

typedef union
{
    float f;
    int32_t i;
}Float2Int32;


const std::string SERVER_IP="192.168.2.248" ;  // 192.168.1.101
const std::string CLIENT_IP="192.168.2.215" ;
const uint16_t SOCKET_PORT= 9527;

constexpr float kMinLaserRange=0.10;
constexpr float kMaxLaserRange=3.0;


}



#endif // COMMON_H
