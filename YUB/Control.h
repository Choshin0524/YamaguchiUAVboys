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
    void MainControl(Sbus* sbus);
    void MotorControl();
private:
    Servo servoObj[SERVO_INDEX];
    Servo ESCObj;
    uint8_t servoOutputPin[SERVO_INDEX];
    uint8_t ESCOutputPin;
    uint16_t servoOutput[SERVO_INDEX];
private:
    // int16_t ---> 16bits  int
    // left aileron control range: 0 - 180, flat: 90 CH1
    uint16_t leftAileronAngle;
    
    // right aileron control range: 0 - 180, flat: 90 CH1
    uint16_t rightAileronAngle;

    // elevator control range: 0 - 180, flat: 90 CH2
    uint16_t elevatorAngle;
    
    // left throttle control range: 0 - 100, max power: 100 CH3
    uint16_t leftThrottle;
    uint16_t thrust;
    
    // side-force Plate control !!not used!! CH4
    uint16_t sideForcePlate;
    
    // rudder control range: 0 - 180, flat: 90 CH5
    uint16_t rudderAngle;

    // right throttle control range: 0 - 100, max power: 100 CH6
    // not be used when single motor
    uint16_t rightThrottleRatio;
};

#endif
