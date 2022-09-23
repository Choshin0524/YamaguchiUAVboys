#include "Control.h"
#include "Arduino.h"

Control::Control()
{
}

void Control::Initialize()
{
    // set servo motor output pin
    servoOutputPin[0] = 32; // leftAileron
    servoOutputPin[1] = 33; // rightAileron
    servoOutputPin[2] = 25; // elevator 
    servoOutputPin[3] = 26; // rudder
    // set brushless motor output pin
    ESCOutputPin = 27;
    
    // attach servo pin
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].attach(servoOutputPin[i]);
    }
    // attach brushless motor output pin
    ESCObj.attach(ESCOutputPin);
    // set esc min/max pulse width
    ESCObj.writeMicroseconds(2824);
    delay(2000);
    ESCObj.writeMicroseconds(1225);
    delay(1000);
}

void Control::MainControl(Sbus* sbus)
{
    // aileron, elevator, rudder
    leftAileronAngle  = map(sbus->GetCh(0), 225, 1820, 0, 180);
    rightAileronAngle = leftAileronAngle;
    elevatorAngle     = map(sbus->GetCh(1), 225, 1820, 0, 180);
    rudderAngle       = map(sbus->GetCh(3), 225, 1820, 0, 180);
    // thrust
    leftThrottle = (sbus->GetCh(2));

    // allocate result to servo output
    servoOutput[0] = leftAileronAngle;
    servoOutput[1] = rightAileronAngle;
    servoOutput[2] = elevatorAngle;
    servoOutput[3] = rudderAngle;

    // output to servo
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].write(servoOutput[i]);
    }
}

void Control::MotorControl()
{
    // allocate thrust to motor output
    thrust = leftThrottle + 1000;
    // output to motor
    ESCObj.writeMicroseconds(thrust);
}