#ifndef Sbus_h
#define Sbus_h
#include "Arduino.h"

class Sbus
{
public:
    Sbus();
    void SbusRead(HardwareSerial& uart);
    int16_t GetCh(unsigned int chNum) const;
    int16_t GetOffset(unsigned int offsetNum) const;
private:
    uint8_t dataBuffer[18];
    int16_t chBuffer[12];
    int16_t offset[8];
};

#endif
