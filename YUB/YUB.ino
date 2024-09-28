#include <math.h>
#include "Sbus.h"
#include "Control.h"
#include "Sensor.h"
// initialize
HardwareSerial SbusSerial(2);
// Adafruit_BNO055 *bno = Adafruit_BNO055(55, 0x28); //bno055 sensor
Sensor *sensor = new Sensor();
Sbus *sbus = new Sbus();     // futaba reciver
Control *ctl = new Control(); // motor output
// ESC initialize flag
bool Initialized = false; // motor output initialize

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
  Serial.begin(115200);
  pinMode(21, INPUT_PULLUP); //SDA PULLUP
  pinMode(22, INPUT_PULLUP); //SCL PULLUP
}

void loop(void)
{
  if (!Initialized)
  {
    sensor->SensorInitalize();
    sensor->SensorCalibration();
    ctl->Initialize();
    Initialized = true;
    Serial.println("Main motor initialized.");
  }
  sensor->SensorRead();
  sensor->DataMonitor(false);
  if (sbus->SbusRead(SbusSerial))
  {
    sbus->DataMonitor(false);
    ctl->DataMonitor(true);
    ctl->MainControl(sbus, sensor);
    ctl->MotorControl(sbus);
  }
}
