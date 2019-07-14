#ifndef MODBUS_H
#define MODBUS_H

#include "paras.h"
#include <qdebug.h>

namespace HL {


#define FIXEDLENGTH     9
#define CRC_OFFSET      4
#define SOCKETBUFMAX 	10000
#define EFLMAX_BYTE		256	// the maximum effective frame length

typedef enum
{
    HEAD=0x5a,
    TAIL=0xa5
}FrameHeadAndTail;

typedef struct ModbusMsg
{
    static const byte head=HEAD;
    ParaGetSet pack;
    uint16_t crc;
    static const byte tail=TAIL;
}ModbusMsg;


class Modbus
{
private:
    ParaGetSet Info;
    uint8_t status;
    uint16_t dat_len;

public:
    Modbus();
    ~Modbus();

public:
    uint16_t PackParas(const ParaGetSet& info ,byte outMsg[]);
    bool UnPackparas(const byte* inMsg, int32_t& startIndex,int32_t endIndex,ParaGetSet& packInfo);
    void SendParas(const ParaGetSet & packInfo ,void write(const char* msg,int64_t size));

private:
    uint16_t CalculateCRC16(const byte  buf[], uint16_t size);
    void InitCRC16_1201_table(void);
    uint16_t CalculateCRC16ByTable(const byte buf[], int32_t start, int32_t end);
};

}



#endif // MODBUS_H
