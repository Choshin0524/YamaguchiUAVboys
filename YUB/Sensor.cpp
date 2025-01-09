#include "Sensor.h"
#include "Arduino.h"
#include <Adafruit_BNO055.h>

Sensor::Sensor()
{
    bno = Adafruit_BNO055(55, 0x28);
    roll = 0.0f; pitch = 0.0f; yaw = 0.0f;
    roll_co = 0.0f; pitch_co = 0.0f; yaw_co = 0.0f;
    x_gyro = 0.0f; y_gyro = 0.0f; z_gyro = 0.0f;
    bno.setExtCrystalUse(false); //no need?
}

void Sensor::SensorInitialize()
{
    // check bno055 begin
    if (!bno.begin())
    {
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    else
    {
        Serial.println("BNO055 detected!");
    }
    delay(100);
}

void Sensor::SensorCalibration()
{

}

void Sensor::SensorRead()
{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    //gyro    
    z_gyro = gyro.z();
    //euler
    yaw = 360 - euler.x();
    roll = -euler.y();
    pitch = euler.z();
    if (pitch >= 0)
    {
        pitch = 180 - pitch;
    }
    else if (pitch <= 0)
    {
        pitch = -(pitch + 180);
    }
}

float Sensor::GetRoll() const
{
    return roll;
}

float Sensor::GetPitch() const
{
    return pitch;
}

float Sensor::GetYaw() const
{
    return yaw;
}

float Sensor::GetYawGyro() const
{
    return z_gyro;
}

void Sensor::DataMonitor(bool ifCheck) const
{
    // Serial print datacheck
    if (ifCheck == true) // if print checker
    {
        Serial.print(roll);
        Serial.print("  ");
        Serial.print(pitch);
        Serial.print("  ");
        Serial.print(yaw);
        Serial.print("  ");
        Serial.print(z_gyro);
        Serial.print("  ");
        Serial.println();
    }
}

void Sensor::DataSDCardOutput(SDCardModule *sdc, File &file, const float &CurSec, const float &pressure)
{
    if (sdc == nullptr)
    {
        Serial.println("SD card error.");
        return;
    }
    sdc->WriteData(file, CurSec);
    sdc->Write(file, ",");
    sdc->WriteData(file, roll);
    sdc->Write(file, ",");
    sdc->WriteData(file, pitch);
    sdc->Write(file, ",");
    sdc->WriteData(file, yaw);
    sdc->Write(file, ",");
    sdc->WriteData(file, z_gyro);
    sdc->Write(file, ",");
    sdc->WriteData(file, pressure);
    sdc->Write(file, "\n");
}
