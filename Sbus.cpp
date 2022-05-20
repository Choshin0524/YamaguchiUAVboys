#include "Sbus.h"

Sbus::Sbus()
{
    HardwareSerial UART2(2); // 02ポートをシリアル通信用にする
    UART2.begin(100000,SERIAL_8E2);
}

void Sbus::SbusRead()
{
    sbusBuffer[3] = NULL;
    sbusBuffer[4] = NULL;
    sbusBuffer[5] = NULL;

    // sbusBuffer[0] check
    while(true)
    {
        if(UART2.available() > 0) 
        {
            sbusBuffer[0] = UART2.read();
        }
        if(sbusBuffer[0] == 0x0f)
        {
            break;
        }
    }

    // sbusBuffer にデータ格納
    unsigned int sbusBufferPos = 1;
    while(UART2.available())
    {
        sbusBuffer[sbusBufferPos++] = UART2.read();
    } 

    ch[0]  = ((sbusBuffer[1]|sbusBuffer[2]<<8)&0x07FF);//ch1
    ch[1]  = ((sbusBuffer[2]>>3|sbusBuffer[3]<<5)&0x07FF);//ch2
    ch[2]  = ((sbusBuffer[3]>>6|sbusBuffer[4]<<2|sbusBuffer[5]<<10)&0x07FF);//ch3
    ch[3]  = ((sbusBuffer[5]>>1|sbusBuffer[6]<<7)&0x07FF);//ch4
    ch[4]  = ((sbusBuffer[6]>>4|sbusBuffer[7]<<4)&0x07FF);//ch5
    ch[5]  = ((sbusBuffer[7]>>7|sbusBuffer[8]<<1|sbusBuffer[9]<<9)&0x07FF);//ch6
    ch[6]  = ((sbusBuffer[9]>>2|sbusBuffer[10]<<6)&0x07FF);//ch7
    ch[7]  = ((sbusBuffer[10]>>5|sbusBuffer[11]<<3)&0x07FF);//ch8
    ch[8]  = ((sbusBuffer[12]|sbusBuffer[13]<<8)&0x07FF);//ch9
    ch[9]  = ((sbusBuffer[13]>>3|sbusBuffer[14]<<5)&0x07FF);
    ch[10] = ((sbusBuffer[14]>>6|sbusBuffer[15]<<2|sbusBuffer[16]<<10)&0x07FF);
    ch[11] = ((sbusBuffer[16]>>1|sbusBuffer[17]<<7)&0x07FF);

    for(unsigned short i = 0; i < 12; i++) 
    {
        ch[i] = 0.89286 * ch[i] + 585.7;
    }

    for (unsigned short i = 12; i > 0; i--)
    {
        ch[i] = ch[i - 1];
    }
    
    if(!sbusFlag && ch[8] >= 1300)
    {
        for(unsigned short i = 1; i <= 7; i++) 
        {
            offset[i] = ch[i];
        }
        sbusFlag = true;
    }
}

unsigned int* Sbus::GetCh()
{
    return ch;
}

unsigned int* Sbus::GetOffset()
{
    return offset;
}