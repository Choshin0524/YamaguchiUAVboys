#include <math.h>
#include "Sbus.h"
#include "Control.h"
#include "Sensor.h"
#include "SDCardModule.h"
#include "SD.h"
#include "FS.h"
// initialize
HardwareSerial SbusSerial(2);
// Adafruit_BNO055 *bno = Adafruit_BNO055(55, 0x28); //bno055 sensor
Sensor *sensor = new Sensor();
Sbus *sbus = new Sbus();                // futaba reciver
Control *ctl = new Control();           // motor output
SDCardModule *sdc = new SDCardModule(); // SDcard module

// ESC initialize flag
bool Initialized = false; // motor output initialize
unsigned long currentMillis = 0;
float currentSecond = 0;

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
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
    sdc->SDCardInitialize("Starting...\n");
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
    currentMillis = millis();
    currentSecond = (float)currentMillis / 1000;
    if (sbus->GetCh(10) == 1696)
    {
      File file1 = SD.open("/flightDataRPY.txt", FILE_APPEND);
      sensor->DataSDCardOutput(sdc, file1, currentSecond);
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
