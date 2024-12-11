#include <math.h>
#include "Sbus.h"
#include "Control.h"
#include "Sensor.h"
#include "Barometer.h"
#include "SDCardModule.h"
#include "SD.h"
#include "FS.h"
#include "SoftwareSerial.h"
// HardwareSerial Initialize
// Sbus 352-1024-1696
HardwareSerial SbusSerial(2);

Sensor *sensor = new Sensor();
Barometer *brm = new Barometer();
Sbus *sbus = new Sbus();                // futaba receiver
Control *ctl = new Control();           // motor output
SDCardModule *sdc = new SDCardModule(); // SDcard module

// ESC initialize flag
bool Initialized = false; // motor output initialize
byte ifInRegion = 0;
unsigned long currentMillis = 0;
float currentSecond = 0.0f;

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
}

void loop(void)
{
  if (!Initialized)
  {
    sensor->SensorInitialize();
    sensor->SensorCalibration();
    brm->BarometerInitialize();
    sdc->SDCardInitialize("Starting...\n");
    ctl->Initialize();
    Initialized = true;
    Serial.println("Main motor initialized.");
  }

  // Ros Serial Receive
  if (RosSerial.available() > 0)
  {
    char incomingByte;
    while (incomingByte != '\n')
    {
          incomingByte = RosSerial.read();
          receivedData += incomingByte;
    }
    altitude = receivedData.toFloat();
    Serial.print("Received altitude: ");
    Serial.println(altitude);
    receivedData = "";
  }
  if (ifInRegion == 1)
  {
    // Serial.println("In Region!");
    ctl->ActiveAutoYaw(true);
  }
  else
  {
    // Serial.println("NOT In Region!");
    ctl->ActiveAutoYaw(false);
  }
  sensor->SensorRead();
  brm->BarometerRead();
  sensor->DataMonitor(false);
  brm->DataMonitor(false);
  if (sbus->SbusRead(SbusSerial))
  {
    sbus->DataMonitor(false);
    ctl->DataMonitor(true);
    ctl->MainControl(sbus, sensor);
    ctl->MotorControl(sbus, brm, altitude);
    currentMillis = millis();
    currentSecond = (float)currentMillis / 1000;
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
    ctl->MotorShutdown(); // shut down motor when no sbus input
  }
}
