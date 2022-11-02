#include "Control.h"
#include "Arduino.h"

Control::Control()
{
    leftAileronAngle = 90;
    rightAileronAngle = 90;
    elevatorAngle = 90;
    rudderAngle = 90;
    leftThrottle = 0;

    PitchError[0] = 0.0f;
    PitchError[1] = 0.0f;
    PitchErrorIntegral = 0.0f;
}

void Control::Initialize()
{
    // set servo motor output pin
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

void Control::MainControl(Sbus *sbus, const float& DeltaTime)
{
    // early return if sbus nullptr
    if(sbus == nullptr)
    {
        return;
    }

    // Auto control switch
    uint16_t AutoControlMode = sbus->GetCh(7);
    switch (AutoControlMode)
    {
    case 255:
        IsAutoPitchControlActive = true;
        break;
    default:
        IsAutoPitchControlActive = false;
        break;
    }

    // aileron
    leftAileronAngle = map(sbus->GetCh(0), 225, 1820, 0, 180);
    rightAileronAngle = leftAileronAngle;
    // elevator
    if (IsAutoPitchControlActive)
    {
        elevatorAngle += (int)AutoControl(SensorPitch, TargetAngle, DeltaTime, KP, KI, KD, PitchError, PitchErrorIntegral);
    }
    else
    {
        elevatorAngle = map(sbus->GetCh(1), 225, 1820, 0, 180);
    }
    // rudder
    rudderAngle = map(sbus->GetCh(3), 225, 1820, 0, 180);
    // aileron offset
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

void Control::MotorControl() const
{
    // allocate thrust to motor output
    int thrust = leftThrottle + 1000;
    // output to motor
    ESCObj.writeMicroseconds(thrust);
}

float Control::AutoControl(const float& SensorValue, const float& TargetValue, const float& DeltaTime,
                              const float& KP, const float& KI, const float& KD,
                              float* Error, float& ErrorIntegral)
{
    float ProportionalOutput, IntegralOutput, DerivativeOutput;
    Error[0] = Error[1];
    Error[1] = SensorValue - TargetValue;
    ErrorIntegral += (Error[1] + Error[0]) / 2.0 * DeltaTime;

    ProportionalOutput = KP * Error[1];
    IntegralOutput = KI * ErrorIntegral;
    DerivativeOutput = KD * (Error[1] - Error[0]) / DeltaTime;

    return (ProportionalOutput + IntegralOutput + DerivativeOutput);
}