/*ESP32に書き出す時，気圧計を抜くこと*/
#include <math.h>
#include "Sbus.h"
#include "Control.h"
#include "Sensor.h"
#include "Barometer.h"
#include "SDCardModule.h"
#include "SD.h"
#include "FS.h"
#include "SoftwareSerial.h"

#define SENSOR_MONITOR_CHECK 0
#define BAROMETER_MONITOR_CHECK 0
#define SBUS_MONITOR_CHECK 0
#define CONTROL_MONITOR_CHECK 0
#define IF_USE_ROS 0
// create instances
HardwareSerial SbusSerial(2);           // for Sbus serial
Sensor *sensor = new Sensor();          // BNO055
Barometer *brm = new Barometer();       // Barometer DPS310
Sbus *sbus = new Sbus();                // futaba receiver
Control *ctl = new Control();           // motor output
SDCardModule *sdc = new SDCardModule(); // SDcard module
// for time counter
float currentSecond = 0.0f;
// ROS
HardwareSerial RosSerial(1);
String receivedData;
float altitude = 0.0f;

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
  RosSerial.begin(115200, SERIAL_8N1, 0, 13);
  Serial.begin(115200);
  pinMode(21, INPUT_PULLUP); // SDA PULLUP
  pinMode(22, INPUT_PULLUP); // SCL PULLUP
  sensor->SensorInitialize();
  sensor->SensorCalibration();
  brm->BarometerInitialize();
  sdc->SDCardInitialize("Starting...\n");
  ctl->Initialize();
  Serial.println("Main motor initialized.");
}

void loop(void)
{
  // Ros Serial Receive for AUTO ALTITUDE
  if (RosSerial.available() > 0 && IF_USE_ROS)
  {
    char incomingByte;
    while (incomingByte != '\n')
    {
      incomingByte = RosSerial.read();
      receivedData += incomingByte;
    }
    if (receivedData.toFloat() > -100.0f) // if altitude ok
    {
      altitude = receivedData.toFloat();
    }
    receivedData = "";
  }

  // BNO055 READ
  sensor->SensorRead();
  sensor->DataMonitor(SENSOR_MONITOR_CHECK);
  // Barometer DPS310 READ
  brm->BarometerRead();
  brm->DataMonitor(BAROMETER_MONITOR_CHECK);

  // MAIN CONTROL
  if (sbus->SbusRead(SbusSerial))
  {
    sbus->DataMonitor(SBUS_MONITOR_CHECK);
    ctl->DataMonitor(CONTROL_MONITOR_CHECK);
    ctl->MainControl(sbus, sensor);
    ctl->MotorControl(sbus, brm, -altitude);
    currentSecond = (float)millis() / 1000;
    if (sbus->GetCh(7) == 1696)
    {
      File file1 = SD.open("/flightDataRPY.txt", FILE_APPEND);
      sensor->DataSDCardOutput(sdc, file1, currentSecond, brm->GetPressure());
      file1.close();
      File file2 = SD.open("/flightDataCTL.txt", FILE_APPEND);
      ctl->DataSDCardOutput(sdc, file2, currentSecond);
      file2.close();
    }
  }
  else
  {
    ctl->MotorShutdown(); // shut down motor when no sbus input **NO WORKING NOW**
  }
}
