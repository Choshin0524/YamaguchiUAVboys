#include "IMU.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

IMU::IMU()
{
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
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

void IMU::IMURead()
{
    // 加速度センサ
    imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    A_x_sen = accelermetor.x();
    A_y_sen = accelermetor.y();
    A_z_sen = accelermetor.z();
    AY = -(float)(A_x_sen * 0.488 * 0.001);
    AX = (float)(A_y_sen * 0.488 * 0.001);
    AZ = (float)(A_z_sen * 0.488 * 0.001);
    //ジャイロセンサ
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    G_x_sen = gyroscope.x();
    G_y_sen = gyroscope.y();
    G_z_sen = gyroscope.z();
    Qsen = -(float)((PI / 180.0) * G_x_sen * 0.03048);
    Psen = (float)((PI / 180.0) * G_y_sen * 0.03048);
    Rsen = (float)((PI / 180.0) * G_z_sen * 0.03048);
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