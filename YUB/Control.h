#ifndef Control_h
#define Control_h
#include "Arduino.h"
#include "ESP32Servo.h"
#include "Sbus.h"
#include "Sensor.h"

#define SERVO_INDEX 5
#define ESC_INDEX 2
#define ALI_KP 2.0f
#define ELE_KP 1.8f
#define RUD_KP 0.5f

class Control
{
public:
    Control();
    void Initialize();                            // set motor output pin & initialize ESC
    void MainControl(Sbus *sbus, Sensor *sensor); // control servo motors
    void MotorControl(Sbus *sbus);                // control thrust
    void MotorShutdown();                         // shutdown motor when no sbus input
    void DataMonitor(bool ifCheck) const;
    void DataSDCardOutput(SDCardModule *sdc, File &file, const float &CurSec);
    void ActiveAutoYaw(bool ifActive);
private:
    Servo servoObj[SERVO_INDEX];
    uint8_t servoOutputPin[SERVO_INDEX];
    uint16_t servoOutput[SERVO_INDEX];

    Servo ESCObj[ESC_INDEX];
    uint8_t ESCOutputPin[ESC_INDEX];

private:
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

    // auto roll,pitch checker
    bool autoRoll;
    bool autoPitch;
    bool autoTakeoffYaw;

    // take-off phase
    bool idle;
    bool takeoff;
    bool takeoffInit;
    bool cruise;
    float currentTime;
    float takeoffTime;

    // auto roll,pitch target angle default->0 deg
    float rollAngleRef;
    float pitchAngleRef;
    float yawRateRef;
};

#endif
