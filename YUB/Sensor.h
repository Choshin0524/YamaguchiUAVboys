#ifndef Sensor_h
#define Sensor_h
#include "Arduino.h"
#include <Adafruit_BNO055.h>
#include "SDCardModule.h"
#include "SD.h"
#include "FS.h"
// BNO055 AKIDUKI

class Sensor
{
private:
    float roll, pitch, yaw;          // sensor output
    float roll_co, pitch_co, yaw_co; // sensor value compensation
    float x_gyro, y_gyro, z_gyro;    // gyroscope sensor value z:yaw
    Adafruit_BNO055 bno;             // bno055 instance   

public:
    Sensor();
    void SensorInitialize();
    void SensorCalibration();
    void SensorRead();                // read sensor value

    float GetRoll() const;            // to get roll value
    float GetPitch() const;           // to get pitch value
    float GetYaw() const;             // to get yaw value

    float GetYawGyro() const;

    void DataMonitor(bool ifCheck) const;
    void DataSDCardOutput(SDCardModule *sdc, File &file, const float &CurSec, const float &pressure);
};

#endif