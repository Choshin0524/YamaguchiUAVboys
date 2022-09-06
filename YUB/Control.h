#ifndef Control_h
#define Control_h
#include "Arduino.h"
#include "Macros.h"
#include <Servo.h>

class Control
{
public:
    Control();
    void MainControl();
private:
    int servoOutputPin[SERVO_INDEX];
    int ESCOutputPin;
    int servoOutput[SERVO_INDEX];
    Servo servoObj[SERVO_INDEX];
    Servo ESCObj;
private:
    // uint8_t ---> 8bits unsigned int (range 0,255)
    // left aileron control range: 0 - 180, flat: 90 CH1
    float leftAileronAngle;
    
    // right aileron control range: 0 - 180, flat: 90 CH1
    float rightAileronAngle;

    // elevator control range: 0 - 180, flat: 90 CH2
    float elevatorAngle;
    
    // left throttle control range: 0 - 100, max power: 100 CH3
    float leftThrottle;
    int thrust;
    
    // side-force Plate control !!not used!! CH4
    float sideForcePlate;
    
    // rudder control range: 0 - 180, flat: 90 CH5
    float rudderAngle;

    // right throttle control range: 0 - 100, max power: 100 CH6
    // not be used when single motor
    float rightThrottleRatio;
};

#endif