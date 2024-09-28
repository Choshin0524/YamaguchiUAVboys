#include "Control.h"
#include "Arduino.h"
#include "YUBMath.h"

Control::Control()
{
  rollAngleRef = 0;
}

void Control::Initialize()
{
    // set servo motor output pin
    servoOutputPin[0] = 26; // leftAileron
    servoOutputPin[1] = 27; // rightAileron
    servoOutputPin[2] = 25; // elevator
    servoOutputPin[3] = 32; // rudder
    servoOutputPin[4] = 33; // side-force Plate

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
        ESCObj[i].writeMicroseconds(1352);
        delay(1500);
    }

    // attach servo pin
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].attach(servoOutputPin[i]);
    }
}

void Control::MainControl(Sbus *sbus, Sensor *sensor)
{
    if (sbus->GetCh(8) >= 1400)
    {
        autoRoll = true;
    }
    else
    {
        autoRoll = false;
    }
    
    // aileron, elevator, rudder, SFP  get
    // map(signal, min, max, minOut, minMax)
    leftAileronAngle = ServoReverse(ServoMap(sbus->GetCh(0), 1696, 352, 0));
    rightAileronAngle = leftAileronAngle;
    elevatorAngle = ServoMap(sbus->GetCh(1), 1648, 373, 70);
    rudderAngle = ServoMap(sbus->GetCh(3), 1696, 352, 70);
    sideForcePlate = ServoReverse(rudderAngle);

    //auto roll
    if (autoRoll)
    {
        leftAileronAngle = ServoReverse(90 - (ALI_KP * (sensor->GetRoll() - rollAngleRef)));
        rightAileronAngle = leftAileronAngle;
    }
    
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

void Control::MotorControl(Sbus *sbus)
{
    // allocate result to thrust
    thrust[0] = sbus->GetCh(2);
    // left thrust = right thrust (SET DIFFRENT IF NEED)
    thrust[1] = thrust[0];
    // output to motor
    for (int i = 0; i < ESC_INDEX; i++)
    {
        ESCObj[i].writeMicroseconds(thrust[i] + 1000);
    }
}

void Control::DataMonitor(bool ifCheck) const
{
    if (ifCheck == true)
    {
        if (autoRoll)
        {
          Serial.print("ROLL AUTO ON   ");
        }
        for (int i = 0; i < SERVO_INDEX; i++)
        {
            Serial.print(i + 1);
            Serial.print(":");
            Serial.print(servoOutput[i]);
            Serial.print(" -- ");
        }
        Serial.println();
    }
}
