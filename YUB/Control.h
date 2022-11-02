#ifndef Control_h
#define Control_h
#include "Arduino.h"
#include <Servo.h>
#include "Sbus.h"

#define SERVO_INDEX 4

class Control
{
public:
    Control();
    void Initialize();
    void MainControl(Sbus* sbus, const float& DeltaTime);
    void MotorControl() const;
private:
    // for auto control (PID)
    float AutoControl(const float& SensorValue, const float& TargetValue, const float& DeltaTime,
                     const float& KP, const float& KI, const float& KD,
                     float* Error, float& ErrorIntegral);
    bool IsAutoPitchControlActive;
    // for pitch auto control
    float PitchError[2];
    float PitchErrorIntegral;
private:
    Servo servoObj[SERVO_INDEX];
    Servo ESCObj;
    uint8_t servoOutputPin[SERVO_INDEX];
    uint8_t ESCOutputPin;
    uint16_t servoOutput[SERVO_INDEX];

public:
    // int16_t ---> 16bits  int
    // left aileron control range: 0 - 180, flat: 90 CH1
    uint16_t leftAileronAngle;
    
    // right aileron control range: 0 - 180, flat: 90 CH1
    uint16_t rightAileronAngle;

    // elevator control range: 0 - 180, flat: 90 CH2
    uint16_t elevatorAngle;
    
    // left throttle control range: 0 - 100, max power: 100 CH3
    uint16_t leftThrottle;
    
    // side-force Plate control !!not used!! CH4
    uint16_t sideForcePlate;
    
    // rudder control range: 0 - 180, flat: 90 CH5
    uint16_t rudderAngle;

    // right throttle control range: 0 - 100, max power: 100 CH6
    // !!not used!! when single motor
    uint16_t rightThrottleRatio;

    uint16_t AileronTakeOffOffset;
    uint16_t AileronLevelFlightOffset;
    uint16_t AileronLandingOffset;
    
};

#endif
