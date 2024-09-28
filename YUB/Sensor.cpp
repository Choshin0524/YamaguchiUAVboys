#include "Sensor.h"
#include "Arduino.h"
#include <Adafruit_BNO055.h>

Sensor::Sensor()
{
    bno = Adafruit_BNO055(55, 0x28);
    bno.setExtCrystalUse(false);
}

void Sensor::SensorInitalize()
{
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
    roll_co, pitch_co, yaw_co = 0.0f;
    for (int i = 0; i < 10; i++)
    {
        SensorRead();
        delay(50);
    }
    yaw_co = yaw;
}

void Sensor::SensorRead()
{
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
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

void Sensor::DataMonitor(bool ifCheck) const
{
    if (ifCheck == true)
    {
        Serial.print(roll);
        Serial.print("  ");
        Serial.print(pitch);
        Serial.print("  ");
        Serial.print(yaw);
        Serial.print("  ");
        Serial.println();
    }
}
