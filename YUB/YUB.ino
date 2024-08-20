#include <math.h>
#include "Sbus.h"
#include "Control.h"
// initialize
<<<<<<< HEAD
HardwareSerial SbusSerial(2);
Sbus* sbus = new Sbus(); // futaba reciver
Control* ctl = new Control(); // motor output

// ESC initialize flags
=======
HardwareSerial SbusSerial(2); // rx:16, tx:17
Sbus *sbus = new Sbus();      // futaba reciver
Control *ctl = new Control(); // motor output
// flags
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22
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
<<<<<<< HEAD

  if (sbus->SbusRead(SbusSerial))
  {
    ctl->MainControl(sbus);
    ctl->MotorControl();
    ctl->DataMonitor();
  }
/*  
=======
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22
  for (int i = 0; i < 12; i++)
  {
    Serial.print(sbus->GetCh(i));
    Serial.print("--");
  }
<<<<<<< HEAD
  
  Serial.println();
*/
=======
  Serial.print(ctl->leftAileronAngle);
  Serial.println();
  if (sbus->SbusRead(SbusSerial))
  {
    ctl->MainControl(sbus);
    ctl->MotorControl();
  }
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22
}
