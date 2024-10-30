#include "Barometer.h"

Barometer::Barometer()
{
    dps_temp = dps.getTemperatureSensor();
    dps_pressure = dps.getPressureSensor();
}

void Barometer::BarometerInitialize()
{
    Wire1.begin(2, 4);
    if (!dps.begin_I2C((119), &Wire1))
    {
        Serial.println("Failed to find Barometer");
        return;
    }
    else
    {
        Serial.println("Barometer OK!");
    }
    dps.configurePressure(DPS310_16HZ, DPS310_16SAMPLES);
    dps.configureTemperature(DPS310_16HZ, DPS310_16SAMPLES);
    if (dps_pressure != nullptr)
    {
        dps_pressure->printSensorDetails();
    }
    else
    {
        return;
    }
}

void Barometer::BarometerRead()
{
    sensors_event_t temp_event, pressure_event;

    if (dps.temperatureAvailable())
    {
        dps_temp->getEvent(&temp_event);
        temp = temp_event.temperature;
    }
    if (dps.pressureAvailable())
    {
        dps_pressure->getEvent(&pressure_event);
        pressure = pressure_event.pressure;
    }
}

void Barometer::DataMonitor(bool ifCheck) const
{
    if (ifCheck == true)
    {
        Serial.print(pressure);
        Serial.print("hPa");
        Serial.println();
    }
}

float Barometer::GetPressure() const
{
    return pressure;
}
