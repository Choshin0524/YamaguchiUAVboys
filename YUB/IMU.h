#ifndef IMU_h
#define IMU_h
#include "Macros.h"
#include "Arduino.h"

class IMU
{
public:
    IMU();
    void IMURead();

    float GetAX() const;
    float GetAY() const;
    float GetAZ() const;

    float GetAXave() const;
    float GetAYave() const;
    float GetAZave() const;

    float GetP() const;
    float GetQ() const;
    float GetR() const;
    
    float GetPsen() const;
    float GetQsen() const;
    float GetRsen() const;
    
    float GetPave() const;
    float GetQave() const;
    float GetRave() const;

private:
    void I2CRead(uint8_t Address,uint8_t Register,uint8_t Byte,uint8_t* Data);
    void I2CWrite(uint8_t Address,uint8_t Register,uint8_t Data);
public:
    uint8_t buffer[25];
    int16_t A_x_sen, A_y_sen, A_z_sen;
    float AX, AY, AZ;
    float AXave, AYave, AZave;
    int16_t G_x_sen, G_y_sen, G_z_sen;
    float P, Q, R;
    float Psen, Qsen, Rsen;
    float Pave, Qave, Rave;
};

#endif