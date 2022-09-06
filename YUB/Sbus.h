#ifndef Sbus_h
#define Sbus_h
#include "Arduino.h"

class Sbus
{
public:
    Sbus();
    void SbusRead(HardwareSerial& uart);
    float GetCh(unsigned int chNum) const;
public:
    uint8_t dataBuffer[18];
    int16_t chBuffer[12];
};

#endif