#include "Control.h"
#include "Arduino.h"
#include "YUBMath.h"

Control::Control()
{
    rollAngleRef = 0.0f;
    pitchAngleRef = -15.0f;
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

    // set esc min/max pulse width
    for (int i = 0; i < ESC_INDEX; i++)
    {
        ESCObj[i].attach(ESCOutputPin[i]);
        Serial.print("Motor ");
        Serial.print(i);
        Serial.println("Initializing....");
        ESCObj[i].writeMicroseconds(2694);
        delay(2000);
        ESCObj[i].writeMicroseconds(1352);
        delay(2000);
    }

    // attach servo pin
    for (int i = 0; i < SERVO_INDEX; i++)
    {
        servoObj[i].attach(servoOutputPin[i]);
    }
}

void Control::MainControl(Sbus *sbus, Sensor *sensor)
{
    // auto control ON/OFF
    if (sbus->GetCh(8) == 1696)
    {
        autoRoll = true;
    }
    else
    {
        autoRoll = false;
    }
    if (sbus->GetCh(11) == 1696)
    {
        autoPitch = true;
    }
    else
    {
        autoPitch = false;
    }
    if (autoPitch && autoRoll && sbus->GetCh(9) == 1696)
    {
        autoThrust = true;
    }
    else
    {
        autoThrust = false;
    }
    

    // aileron, elevator, rudder, SFP  get
    // map(signal, min, max, minOut, minMax)
    leftAileronAngle = ServoReverse(ServoMap(sbus->GetCh(0), 1696, 352, 0));
    rightAileronAngle = leftAileronAngle;
    elevatorAngle = ServoMap(sbus->GetCh(1), 1648, 373, 70);
    rudderAngle = ServoMap(sbus->GetCh(3), 1696, 352, 55);
    sideForcePlate = ServoReverse(rudderAngle);

    // auto roll
    if (autoRoll)
    {
        leftAileronAngle = ServoReverse(90 - (ALI_KP * (sensor->GetRoll() - rollAngleRef)));
        rightAileronAngle = leftAileronAngle;
    }
    // auto pitch
    if (autoPitch)
    {
        elevatorAngle = (90 - (ELE_KP * (sensor->GetPitch() - pitchAngleRef)));
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
        if (servoOutput[i] >= 0 && servoOutput[i] <= 180)
        {
            servoObj[i].write(servoOutput[i]);
        }
        else // exception prevent
        {
            servoObj[i].write(90);
        }
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
            Serial.print("**ROLL AUTO ON**");
        }
        if (autoPitch)
        {
            Serial.print("**PITCH AUTO ON**");
        }
        if (autoThrust)
        {
            Serial.print("**THRUST AUTO ON**");
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

void Control::DataSDCardOutput(SDCardModule *sdc, File &file, const float CurSec)
{
    sdc->WriteData(file,  CurSec);
    sdc->Write(file,  ",");
    sdc->WriteData(file,leftAileronAngle);
    sdc->Write(file,",");
    sdc->WriteData(file,rightAileronAngle);
    sdc->Write(file,",");
    sdc->WriteData(file, elevatorAngle);
    sdc->Write(file, ",");
    sdc->WriteData(file, thrust[0]);
    sdc->Write(file, ",");
    sdc->WriteData(file, thrust[1]);
    sdc->Write(file, ",");
    sdc->WriteData(file, rudderAngle);
    sdc->Write(file, ",");
    sdc->WriteData(file, autoRoll);
    sdc->Write(file, ",");
    sdc->WriteData(file, autoPitch);
    sdc->Write(file, "\n");
}
