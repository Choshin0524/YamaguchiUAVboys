#include "Control.h"
#include "Arduino.h"
#include "YUBMath.h"
#include <math.h>

Control::Control()
{
    autoRoll = false;
    autoPitch = false;
    autoTakeoffYaw = false;

    IfRosTrue = false;
    idle = false;
    takeoff = false;
    takeoffInit = false;
    cruise = false;

    altitudeRef = 3.0f;
    fixedAltitude = 0.0f;

    currentTime = 0.0f;
    takeoffTime = 0.0f;
    takeoffPressure = 0.0f;
    prevPressure = 1000.0f;
    pressure_diff = 0.0f;
    pressureFixCount = 0;
    pressureFixSum = 0.0f;
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
    ESCOutputPin[0] = 14; //right motor
    ESCOutputPin[1] = 12; //left motor

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

    // THRUST
    if (autoPitch && autoRoll && sbus->GetCh(10) == 352) // switch->1
    {
        idle = false;
        takeoff = false;
        cruise = false;
    }
    else if (autoPitch && autoRoll && sbus->GetCh(10) == 1024) // switch->2
    {
        idle = true;
        takeoff = false;
        cruise = false;
    }
    else if (autoPitch && autoRoll && sbus->GetCh(10) == 1696) // switch->3
    {
        idle = false;
        currentTime = (float)millis() / 1000;
        if (!takeoffInit)
        {
            takeoff = true;
            takeoffTime = currentTime;
            takeoffInit = true;
            autoTakeoffYaw = true;
            thrust[0] = 500;
            thrust[1] = 500;
        }
        if (currentTime - takeoffTime >= 2.0f) //take off thrust time
        {
            takeoff = false;
            cruise = true;
            thrust[0] = 920;
            thrust[1] = 920;
            autoTakeoffYaw = false;
        }
    }

    // aileron, elevator, rudder, SFP  get
    // map(signal, min, max, minOut, minMax)
    leftAileronAngle = ServoReverse(ServoMap(sbus->GetCh(0), 1696, 352, 0));
    rightAileronAngle = leftAileronAngle;
    elevatorAngle = ServoMap(sbus->GetCh(1), 1696, 352, 70);
    rudderAngle = ServoMap(sbus->GetCh(3), 1696, 352, 58);
    sideForcePlate = rudderAngle;

    // auto roll
    if (autoRoll)
    {
        leftAileronAngle = 90 - (ALI_KP * (sensor->GetRoll() - ROLL_ANGLE_REF));
        if (leftAileronAngle <= 40)
        {
            leftAileronAngle = 40;
        }
        if (leftAileronAngle >= 140)
        {
            leftAileronAngle = 140;
        }
        leftAileronAngle = ServoReverse(leftAileronAngle);
        rightAileronAngle = leftAileronAngle;
    }
    // auto pitch
    if (autoPitch)
    {
        elevatorAngle = (90 - (ELE_KP * (sensor->GetPitch() - PITCH_ANGLE_REF)));
        if (elevatorAngle <= 30)
        {
            elevatorAngle = 30;
        }
        if (elevatorAngle >= 150)
        {
            elevatorAngle = 150;
        }
        elevatorAngle = ServoReverse(elevatorAngle);
    }

    if (autoTakeoffYaw)
    {
        rudderAngle = (90 + (RUD_KP * (sensor->GetYawGyro() - YAW_RATE_REF)));
        if (rudderAngle <= 40)
        {
            rudderAngle = 40;
        }
        if (rudderAngle >= 140)
        {
            rudderAngle = 140;
        }
        sideForcePlate = rudderAngle;
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

void Control::MotorControl(Sbus *sbus, Barometer *brm, float altitude)
{
    ROSaltitude = altitude; // write altitude argument to member variable
    if (sbus == nullptr)    // check null pointer
    {
        Serial.println("Sbus error.");
        return;
    }
    if (!takeoff)           // not reading from sbus when take off phase
    {
        thrust[0] = sbus->GetCh(2);
        thrust[1] = thrust[0];
    }
    if (idle)
    {
        thrust[0] = 450;
        thrust[1] = 450;
        takeoffPressure = brm->GetPressure();
        // set to prev pressure when barometer data wrong
        if (takeoffPressure > 950.0f && takeoffPressure < 1100.0f)
        {
            prevPressure = takeoffPressure;
        }
        else
        {
            takeoffPressure = prevPressure;
        }
    }
    else if (takeoff)
    {
        if (thrust[0] < 1400)
        {
            thrust[0] += 100; // gradually increase thrust to prevent motor shutdown
        }
        thrust[1] = thrust[0];
    }
    else if (cruise)
    {
        float fixedPressure = brm->GetPressure();

        if (pressureFixCount > 3)
        {
            pressureFixSum = 0.0f; // reset pressure diff sum
            pressureFixCount = 0;  // reset count
        }
        if (fixedPressure > 950.0f && fixedPressure < 1100.0f)
        {
            pressure_diff = -(fixedPressure - prevPressure);
            prevPressure = fixedPressure;
        }
        else
        {
            fixedPressure = prevPressure; // set to prev pressure when barometer data wrong
            pressure_diff = 0.0f;
        }
        pressureFixSum += pressure_diff;
        pressureFixCount++;
        fixedAltitude = ROSaltitude + 10.0f * pressureFixSum;
        // fixedPressure = fixedPressure - (-1.38 * 800 * pow(10, -4) + 4.4 * pow(800, 2) * pow(10, -7) - 1.3 * pow(800, 3) * pow(10, -10));
        thrust[0] = thrust[0] - THU_KP * (fixedAltitude - altitudeRef);
        // thrust[0] = thrust[0] + THU_KP * (fixedPressure - (takeoffPressure + 0.08 - 0.27)) + THU_RUD_KP * abs(90 - rudderAngle);
        thrust[1] = thrust[0];
        thrust[0] += THU_RUD_KP_R * abs(90 - rudderAngle);
        thrust[1] += THU_RUD_KP_L * abs(90 - rudderAngle);
        if (thrust[0] > 1180)
        {
            thrust[0] = 1180;
        }
        if (thrust[1] > 1180)
        {
            thrust[1] = 1180;
        }
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
        Serial.print(thrust[0]);
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
    sdc->Write(file, ",");
    sdc->WriteData(file, IfRosTrue);
    sdc->Write(file, ",");
    sdc->WriteData(file, ROSaltitude);
    sdc->Write(file, ",");
    sdc->WriteData(file, pressureFixSum);
    sdc->Write(file, "\n");
}

void Control::ActiveAutoYaw(bool ifActive)
{
    IfRosTrue = ifActive;
}
