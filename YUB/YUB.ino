#include <math.h>
#include "Sbus.h"
#include "Control.h"
// initialize
HardwareSerial SbusSerial(2);
Sbus* sbus = new Sbus();
Control* ctl = new Control();
bool ctlInitialized = false;
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
  }
  Serial.print("OK");
  sbus->SbusRead(SbusSerial); // フタバ工業：sbus規格デジタル信号を読み込む
  for (int i = 0; i < 12; i++)
  {
    Serial.print(sbus->GetCh(i), DEC);
    Serial.print(" ");
  }
  Serial.println(" ");
  ctl->MainControl(sbus);
  ctl->MotorControl();
}
