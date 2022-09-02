#include <math.h>
#include "Sbus.h"

// initialize
HardwareSerial SbusSerial(2);
Sbus sbus;

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
  Serial.begin(115200);
}

void loop(void)
{
  sbus.SbusRead(SbusSerial); // フタバ工業：sbus規格デジタル信号を読み込む
  for (int i = 0; i < 12; i++)
  {
    Serial.print(sbus.GetCh(3), DEC);
    Serial.print(" ");
  }
  Serial.println(" ");
}
