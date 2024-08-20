#ifndef Control_h
#define Control_h
#include "Arduino.h"
#include <ESP32Servo.h>
#include "Sbus.h"

#define SERVO_INDEX 5
#define ESC_INDEX 2

class Control
{
public:
    Control();
    void Initialize(); //set motor output pin & initialize ESC
    void MainControl(Sbus* sbus); //control servo motors
    void MotorControl();  // control thrust
    void DataMonitor() const;
private:
    Servo servoObj[SERVO_INDEX];
    uint8_t servoOutputPin[SERVO_INDEX];
    uint16_t servoOutput[SERVO_INDEX];
<<<<<<< HEAD

    Servo ESCObj[ESC_INDEX];
    uint8_t ESCOutputPin[ESC_INDEX];

private:
=======
public:
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22
    // int16_t ---> 16bits  int
    // left aileron control range: 0 - 180, flat: 90 CH1
    uint16_t leftAileronAngle;
    
    // right aileron control range: 0 - 180, flat: 90 CH1
    uint16_t rightAileronAngle;

    // elevator control range: 0 - 180, flat: 90 CH2
    uint16_t elevatorAngle;
    
    // left+right throttle control range: 0 - 100, max power: 100 CH3
    uint16_t thrust[2];
    
    // side-force Plate control CH4
    uint16_t sideForcePlate;
    
    // rudder control range: 0 - 180, flat: 90 CH5
    uint16_t rudderAngle;

<<<<<<< HEAD
=======
    // right throttle control range: 0 - 100, max power: 100 CH6
    // !!not used!! when single motor
    uint16_t rightThrottleRatio;

    uint16_t AileronTakeOffOffset;
    uint16_t AileronLevelFlightOffset;
    uint16_t AileronLandingOffset;
    
>>>>>>> 6e90bda8fcb1d22af72ee86ef761169ab64f4a22
};

#endif
