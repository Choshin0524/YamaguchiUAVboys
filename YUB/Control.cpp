#include "Control.h"
#include "Arduino.h"
#include "YUBMath.h"

Control::Control()
{
}

void Control::Initialize()
{
    // set servo motor output pin
<<<<<<< HEAD
    servoOutputPin[0] = 26; // leftAileron
    servoOutputPin[1] = 27; // rightAileron
    servoOutputPin[2] = 25; // elevator 
    servoOutputPin[3] = 32; // rudder
    servoOutputPin[4] = 33;  // side-force Plate
    
    // set brushless motor output pin
    ESCOutputPin[0] = 14;
    ESCOutputPin[1] = 12;

    // attach brushless motor output pin

    // set esc min/max pulse width
    for (int i = 0; i < ESC_INDEX; i++)
    {
      ESCObj[i].attach(ESCOutputPin[i]);
      ESCObj[i].writeMicroseconds(2694);
      delay(2000);
      ESCObj[i].writeMicroseconds(1404);
      delay(1500);
    }
=======
    servoOutputPin[0] = 32; // leftAileron
    servoOutputPin[1] = 33; // rightAileron
    servoOutputPin[2] = 25; // elevator
    servoOutputPin[3] = 26; // rudder
    // set brushless motor output pin
    ESCOutputPin = 14;

    // set aileron offset
    AileronTakeOffOffset = 10;
    AileronLevelFlightOffset = 0;
    AileronLandingOffset = 20;
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22

    // attach servo pin
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].attach(servoOutputPin[i]);
    }
}

void Control::MainControl(Sbus *sbus)
{
<<<<<<< HEAD
    // aileron, elevator, rudder, SFP  get
    // map(signal, min, max, minOut, minMax)
    leftAileronAngle  = ServoMap(sbus->GetCh(0), 1696, 352, 0);
    rightAileronAngle = 180 - leftAileronAngle;
    elevatorAngle     = ServoMap(sbus->GetCh(1), 1360, 688, 40);
    rudderAngle       = ServoMap(sbus->GetCh(3), 1639, 352, 40);
    sideForcePlate    = 180 - rudderAngle;

    // allocate result to thrust
    thrust[0] = sbus->GetCh(2);
    // left thrust = right thrust (SET DIFFRENT IF NEED)
    thrust[1] = thrust[0];
=======
    // aileron, elevator, rudder
    leftAileronAngle = map(sbus->GetCh(0), 225, 1820, 0, 180);
    rightAileronAngle = leftAileronAngle;
    elevatorAngle = map(sbus->GetCh(1), 225, 1820, 0, 180);
    rudderAngle = map(sbus->GetCh(3), 225, 1820, 0, 180);

    uint16_t OffsetMode = sbus->GetCh(6);
    switch (OffsetMode)
    {
    case 1024:
        leftAileronAngle += AileronTakeOffOffset;
        rightAileronAngle -= AileronTakeOffOffset;
        break;
    case 255:
        leftAileronAngle += AileronLevelFlightOffset;
        rightAileronAngle -= AileronLevelFlightOffset;
        break;
    case 1824:
        leftAileronAngle += AileronLandingOffset;
        rightAileronAngle -= AileronLandingOffset;
        break;
    default:
        break;
    }

    // thrust
    leftThrottle = (sbus->GetCh(2));
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22

    // allocate result to servo output
    servoOutput[0] = leftAileronAngle;
    servoOutput[1] = rightAileronAngle;
    servoOutput[2] = elevatorAngle;
    servoOutput[3] = rudderAngle;
    servoOutput[4] = sideForcePlate;

    // output to servo
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].write(servoOutput[i]);
    }
}

void Control::MotorControl()
{
    // output to motor
<<<<<<< HEAD
    for (int i = 0; i < ESC_INDEX; i++)
    {
        ESCObj[i].writeMicroseconds(thrust[i]+1000);
    }
}

void Control::DataMonitor() const
{
    
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        Serial.print(i + 1);
        Serial.print(" : ");
        Serial.print(servoOutput[i]);
        Serial.print("-");
    }
    Serial.println();
    
}
=======
    ESCObj.writeMicroseconds(thrust);
}
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22
