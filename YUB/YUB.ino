#include <math.h>
#include "Sbus.h"
#include "Control.h"

// initialize
HardwareSerial SbusSerial(2); // rx:16, tx:17
Sbus* sbus = new Sbus(); // futaba reciver
Control* ctl = new Control(); // motor output

// flags
bool ctlInitialized = false; // motor output initialize

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
  Serial.begin(115200);
}

void loop(void)
{
  if (!ctlInitialized)
  {
    ctl->Initialize();
    ctlInitialized = true;
    Serial.println("Main motor initialized.");
  }
  sbus->SbusRead(SbusSerial);
  ctl->MainControl(sbus);
  ctl->MotorControl();
}