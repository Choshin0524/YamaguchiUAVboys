#include "IMU.h"
#include "Arduino.h"
#include <Wire.h>

IMU::IMU()
{
    Wire.begin();
    I2CWrite(MPU6050, 27, 0x10);
    I2CWrite(MPU6050, 28, 0x18);
    I2CWrite(MPU6050, 0x37, 0x02);
    I2CWrite(MPU6050, 0x6B, 0x00);
    for (unsigned short i = 0; i < 1000; i++)
    {
        IMURead();
        AXave += AX / 1000;
        AYave += AY / 1000;
        AZave += AZ / 1000;
        Pave += Psen / 1000;
        Qave += Qsen / 1000;
        Rave += Rsen / 1000;
    }
}

void IMU::I2CRead(uint8_t Address, uint8_t Register, uint8_t Byte, uint8_t *Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    Wire.requestFrom(Address, Byte);
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}

void IMU::I2CWrite(uint8_t Address, uint8_t Register, uint8_t Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

void IMU::IMURead()
{
    I2CWrite(MPU6050, 0x1C, 0x18);
    I2CWrite(MPU6050, 0x1D, 0x06);
    I2CWrite(MPU6050, 0x1B, 0x10);
    I2CWrite(MPU6050, 0x1A, 0x06);
    I2CRead(MPU6050, 0x3B, 14, buffer);

    A_x_sen = (buffer[0] << 8 | buffer[1]);
    A_y_sen = (buffer[2] << 8 | buffer[3]);
    A_z_sen = buffer[4] << 8 | buffer[5];

    AY = -(double)(A_x_sen * 0.488 * 0.001);
    AX = (double)(A_y_sen * 0.488 * 0.001);
    AZ = (double)(A_z_sen * 0.488 * 0.001);

    G_x_sen = (buffer[8] << 8 | buffer[9]);
    G_y_sen = (buffer[10] << 8 | buffer[11]);
    G_z_sen = buffer[12] << 8 | buffer[13];

    Qsen = -(double)((PI / 180.0) * G_x_sen * 0.03048);
    Psen = (double)((PI / 180.0) * G_y_sen * 0.03048);
    Rsen = (double)((PI / 180.0) * G_z_sen * 0.03048);
}

// Get AX,AY,AZ
float IMU::GetAX() const
{
    return AX;
}
float IMU::GetAY() const
{
    return AY;
}
float IMU::GetAZ() const
{
    return AZ;
}

// Get AXave,AYave,AZave
float IMU::GetAXave() const
{
    return AXave;
}
float IMU::GetAYave() const
{
    return AYave;
}
float IMU::GetAZave() const
{
    return AZave;
}

// Get P,Q,R
float IMU::GetP() const
{
    return P;
}
float IMU::GetQ() const
{
    return Q;
}
float IMU::GetR() const
{
    return R;
}

// Get Psen,Qsen,Rsen
float IMU::GetPsen() const
{
    return Psen;
}
float IMU::GetQsen() const
{
    return Qsen;
}
float IMU::GetRsen() const
{
    return Rsen;
}

// Get Pave,Qave,Rave
float IMU::GetPave() const
{
    return Pave;
}
float IMU::GetQave() const
{
    return Qave;
}
float IMU::GetRave() const
{
    return Rave;
}