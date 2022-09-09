#include "Control.h"
#include "Arduino.h"

Control::Control()
{
}
void Control::Initialize()
{
   // set servo motor output pin
    servoOutputPin[0] = 32;
    servoOutputPin[1] = 33;
    servoOutputPin[2] = 25;
    servoOutputPin[3] = 26;
    // set brushless motor output pin
    ESCOutputPin = 27;
    
    // attach servo pin
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].attach(servoOutputPin[i]);
    }
    // attach brushless motor output pin
    ESCObj.attach(ESCOutputPin);

    Serial.println("starting...");
    // set esc min/max pulse width
    ESCObj.writeMicroseconds(2824);
        Serial.println("starting..2.");
    delay(2000);
    ESCObj.writeMicroseconds(1225);
        Serial.println("starting...3");
    delay(1000);
}
void Control::MainControl(Sbus* sbus)
{
    // let servo angle = 0 to maximum left range
    // aileron, elevator, rudder
    leftAileronAngle  = map(sbus->GetCh(0), 225, 1820, 0, 180);
    rightAileronAngle = 180 - leftAileronAngle;
    elevatorAngle     = (sbus->GetCh(1) / 9);
    rudderAngle       = (sbus->GetCh(3) / 9);
    // thrust
    leftThrottle = (sbus->GetCh(2));

    // allocate result to servo output
    servoOutput[0] = leftAileronAngle;
    servoOutput[1] = rightAileronAngle;
    servoOutput[2] = elevatorAngle;
    servoOutput[3] = rudderAngle;
    // allocate thrust to motor output
    thrust = leftThrottle + 1000;

    // output to servo
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].write(servoOutput[i]);
    }

}

void Control::MotorControl()
{
      // output to motor
    ESCObj.writeMicroseconds(thrust);
}
