#ifndef Sbus_h
#define Sbus_h
#include "Arduino.h"

class Sbus
{
public:
    Sbus(HardwareSerial& uart);
    void SbusRead(HardwareSerial& uart);
    int16_t GetCh(unsigned int chNum) const;
    int16_t GetOffset(unsigned int offsetNum) const;
private:
    bool sbusFlag = false;
    uint8_t buffer[18];
    int16_t ch[12];
    int16_t offset[8];
};

#endif