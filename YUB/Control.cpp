#include "Control.h"
#include "Arduino.h"
#include "YUBMath.h"

Control::Control()
{
    rollAngleRef = 0.0f;
    pitchAngleRef = -15.0f;
    yawRateRef = 0.0f;
    autoRoll = false;
    autoPitch = false;
    autoTakeoffYaw = false;

    idle = false;
    takeoff = false;
    takeoffInit = false;
    cruise = false;
    currentTime = 0.0f;
    takeoffTime = 0.0f;
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
        ESCObj[i].writeMicroseconds(2696);
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
    if (sbus == nullptr || sensor == nullptr)
    {
        Serial.println("Sbus or sensor error.");
        return;
    }
    // auto control ON/OFF
    // ROLL
    if (sbus->GetCh(8) == 1696)
        autoRoll = true;
    else
        autoRoll = false;
    // PITCH
    if (sbus->GetCh(11) == 1696)
        autoPitch = true;
    else
        autoPitch = false;
    // TAKEOFFYAW
    if (sbus->GetCh(6) == 1696)
        autoTakeoffYaw = true;
    else
        autoTakeoffYaw = false;

    // THRUST
    if (autoPitch && autoRoll && sbus->GetCh(10) == 352) //switch->1
    {
        idle = false;
        takeoff = false;
        cruise = false;
    }
    else if (autoPitch && autoRoll && sbus->GetCh(10) == 1024) //switch->2
    {
        idle = true;
        takeoff = false;
        cruise = false;
    }
    else if (autoPitch && autoRoll && sbus->GetCh(10) == 1696) //switch->3
    {
        idle = false;
        currentTime = (float)millis() / 1000;
        if (!takeoffInit)
        {
            takeoff = true;
            takeoffTime = currentTime;
            takeoffInit = true;
        }
        if (currentTime - takeoffTime >= 2.0f)
        {
            takeoff = false;
            cruise = true;
        }
    }

    // aileron, elevator, rudder, SFP  get
    // map(signal, min, max, minOut, minMax)
    leftAileronAngle = ServoReverse(ServoMap(sbus->GetCh(0), 1696, 352, 0));
    rightAileronAngle = leftAileronAngle;
    elevatorAngle = ServoMap(sbus->GetCh(1), 1696, 352, 70);
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
        if (elevatorAngle <= 30)
        {
            elevatorAngle = 30;
        }
        if (elevatorAngle >= 150)
        {
            elevatorAngle = 150;
        }
    }

    if (autoTakeoffYaw)
    {
        rudderAngle = (90 + (RUD_KP * (sensor->GetYawGyro() - yawRateRef)));
        if (rudderAngle <= 40)
        {
            rudderAngle = 40;
        }
        if (rudderAngle >= 140)
        {
            rudderAngle = 140;
        }
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
    if (sbus == nullptr)
    {
        Serial.println("Sbus error.");
        return;
    }
    // allocate result to thrust
    thrust[0] = sbus->GetCh(2);
    // left thrust = right thrust (SET DIFFRENT IF NEED)
    thrust[1] = thrust[0];
    // output to motor
    
    // take-off
    if (idle)
    {
        thrust[0] = 690;
        thrust[1] = 690;
    }
    else if (takeoff)
    {
        thrust[0] = 1600;
        thrust[1] = 1600;
    }
    else if (cruise)
    {
        thrust[0] = 1100;
        thrust[1] = 1100;
    }
    

    for (int i = 0; i < ESC_INDEX; i++)
    {
        ESCObj[i].writeMicroseconds(thrust[i] + 1000);
    }
}

void Control::MotorShutdown()
{
    for (int i = 0; i < ESC_INDEX; i++)
    {
        ESCObj[i].writeMicroseconds(0);
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
        if (idle)
        {
            Serial.print("**IDLE ON**");
        }
        if (takeoff)
        {
            Serial.print("**TAKEOFF ON**");
        }
        if (cruise)
        {
            Serial.print("**CRUISE ON**");
        }
        if (autoTakeoffYaw)
        {
            Serial.print("**TAKEOFFYAW AUTO ON**");
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

void Control::DataSDCardOutput(SDCardModule *sdc, File &file, const float &CurSec)
{
    if (sdc == nullptr)
    {
        Serial.println("SD card error.");
        return;
    }
    sdc->WriteData(file, CurSec);
    sdc->Write(file, ",");
    sdc->WriteData(file, leftAileronAngle);
    sdc->Write(file, ",");
    sdc->WriteData(file, rightAileronAngle);
    sdc->Write(file, ",");
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
    sdc->Write(file, ",");
    sdc->WriteData(file, autoTakeoffYaw);
    sdc->Write(file, "\n");
}

void Control::ActiveAutoYaw(bool ifActive)
{
}
