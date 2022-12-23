#include <math.h>
#include "Sbus.h"
#include "Control.h"
#include "SDData.h"
// initialize
HardwareSerial SbusSerial(2); // rx:16, tx:17
Sbus *sbus = new Sbus();      // futaba reciver
Control *ctl = new Control(); // motor output
// flags
bool ctlInitialized = false; // motor output initialize

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
  Serial.begin(115200);

  // sdcard init
  SD.begin();
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  SDData::listDir(SD, "/", 0);
  SDData::deleteFile(SD, "/init.txt");
  SDData::appendFile(SD, "/init.txt", "ryota init OK!!!");
  SDData::appendFile(SD, "/data.txt", "\n------\nStarting data record\n------\n");
}

void loop(void)
{
  if (!ctlInitialized)
  {
    ctl->Initialize();
    ctlInitialized = true;
    Serial.println("Main motor initialized.");
  }
  for (int i = 0; i < 12; i++)
  {
    Serial.print(sbus->GetCh(i));
    Serial.print("--");
  }
  Serial.print(ctl->leftAileronAngle);
  Serial.println();
  if (sbus->SbusRead(SbusSerial))
  {
    float passedTime = (float)millis() / 1000.0f;
    String passedTimeStr = String(passedTime, 3);
    String servo1Str = String(ctl->leftAileronAngle);
    String servo2Str = String(ctl->rightAileronAngle);

    ctl->MainControl(sbus);
    ctl->MotorControl();
    SDData::appendFile(SD, "/data.txt", passedTimeStr.c_str());
    SDData::appendFile(SD, "/data.txt", "s: Elevator:");
    SDData::appendFile(SD, "/data.txt", servo1Str.c_str());
    SDData::appendFile(SD, "/data.txt", "degree, Rudder:");
    SDData::appendFile(SD, "/data.txt", servo2Str.c_str());
    SDData::appendFile(SD, "/data.txt", "degree\n");
  }
}
