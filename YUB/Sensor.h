#ifndef Sensor_h
#define Sensor_h
#include "Arduino.h"
#include <Adafruit_BNO055.h>

// BNO055 AKIDUKI

class Sensor
{
private:
    float roll, pitch, yaw; //sensor output
    float roll_co, pitch_co, yaw_co;  //sensor value compensation
    Adafruit_BNO055 bno;
public:
    Sensor();
    void SensorInitalize();
    void SensorCalibration();
    void SensorRead();
    
    float GetRoll() const;
    float GetPitch() const;
    float GetYaw() const;
    
    void DataMonitor(bool ifCheck) const;
};

#endif