#ifndef Barometer_h
#define Barometer_h

#include "Arduino.h"
#include <Adafruit_DPS310.h>
#include <Wire.h>

class Barometer
{
private:
    Adafruit_DPS310 dps;
    Adafruit_Sensor *dps_temp;
    Adafruit_Sensor *dps_pressure;
    float temp;
    float pressure;
public:
    Barometer();
    void BarometerInitialize();
    void BarometerRead();
    void DataMonitor(bool ifCheck) const;
    float GetPressure() const;
};
#endif
