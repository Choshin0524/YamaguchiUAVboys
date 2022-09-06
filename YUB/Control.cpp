#include "Control.h"
#include "Arduino.h"
#include "Sbus.h"

Control::Control()
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

    // set esc min/max pulse width
    ESCObj.writeMicroseconds(MAX_PULSE_WIDTH);
    delay(2000);
    ESCObj.writeMicroseconds(MIN_PULSE_WIDTH);
    delay(2000);
}
void Control::MainControl(Sbus &sbus)
{
    // let servo angle = 0 to maximum left range
    // aileron, elevator, rudder
    leftAileronAngle  = (int)(180 - ((sbus.GetCh(1)) / 8.86));
    rightAileronAngle = 180 - leftAileronAngle;
    elevatorAngle     = (int)(sbus.GetCh(2) / 8.86);
    rudderAngle       = (int)(sbus.GetCh(5) / 8.86);
    // thrust
    leftThrottle = (int)(sbus.GetCh(3));

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
        servoOutput[i] = servoOutput > 108 ? 108 : servoOutput[i]; 
        servoOutput[i] = servoOutput < 51 ? 51 : servoOutput[i]; 
        servoObj[i].write(servoOutput);
    }
    // output to motor
    ESCObj.writeMicroseconds(thrust);
}