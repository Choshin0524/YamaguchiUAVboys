#include <math.h>
#include "Sbus.h"
#include "Control.h"

// initialize Sbus reciver class
HardwareSerial SbusSerial(2);
Sbus sbus;

// initialize control class
Control ctl;

void setup(void)
{
  SbusSerial.begin(100000, SERIAL_8E2);
  Serial.begin(115200);
}

void loop(void)
{
  // Sbus signal read
  sbus.SbusRead(SbusSerial);
  // serial print recived data
  for (int i = 0; i < 12; i++)
  {
    Serial.print(sbus.GetCh(3), DEC);
    Serial.print(" ");
  }
  Serial.println(" ");
  ctl.MainControl(sbus);
}
