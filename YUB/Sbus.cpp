#include "Sbus.h"
#include "Arduino.h"

Sbus::Sbus(HardwareSerial& uart)
{
    uart.begin(100000,SERIAL_8E2);
}

void Sbus::SbusRead(HardwareSerial& uart)
{
     buffer[3] = NULL;
     buffer[4] = NULL;
     buffer[5] = NULL;

    //  buffer[0] check
    while(true)
    {
        if(uart.available() > 0) 
        {
             buffer[0] = uart.read();
        }
        if( buffer[0] == 0x0f)
        {
            break;
        }
    }

    //  buffer にデータ格納
    unsigned int bufferPos = 1;
    while(uart.available())
    {
         buffer[bufferPos++] = uart.read();
    } 

    ch[0]  = ((buffer[1] | buffer[2]<<8)&0x07FF);//ch1
    ch[1]  = ((buffer[2]>>3 | buffer[3]<<5)&0x07FF);//ch2
    ch[2]  = ((buffer[3]>>6 | buffer[4]<<2| buffer[5]<<10)&0x07FF);//ch3
    ch[3]  = ((buffer[5]>>1 | buffer[6]<<7)&0x07FF);//ch4
    ch[4]  = ((buffer[6]>>4 | buffer[7]<<4)&0x07FF);//ch5
    ch[5]  = ((buffer[7]>>7 | buffer[8]<<1| buffer[9]<<9)&0x07FF);//ch6
    ch[6]  = ((buffer[9]>>2 | buffer[10]<<6)&0x07FF);//ch7
    ch[7]  = ((buffer[10]>>5 | buffer[11]<<3)&0x07FF);//ch8
    ch[8]  = ((buffer[12] | buffer[13]<<8)&0x07FF);//ch9
    ch[9]  = ((buffer[13]>>3 | buffer[14]<<5)&0x07FF);
    ch[10] = ((buffer[14]>>6 | buffer[15]<<2| buffer[16]<<10)&0x07FF);
    ch[11] = ((buffer[16]>>1 | buffer[17]<<7)&0x07FF);

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

int16_t Sbus::GetCh(unsigned int chNum) const
{
    return ch[chNum];
}

int16_t Sbus::GetOffset(unsigned int offsetNum) const
{
    return offset[offsetNum];
}