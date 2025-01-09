#ifndef Control_h
#define Control_h
#include "Arduino.h"
#include "ESP32Servo.h"
#include "Sbus.h"
#include "Sensor.h"
#include "Barometer.h"

#define SERVO_INDEX 5 // Servo motor num
#define ESC_INDEX 2   // brushless motor num

// AUTO THRUST PARAM
#define THU_RUD_KP_L 7.5f
#define THU_RUD_KP_R 5.5f
#define THU_KP 80.0f
// AUTO ROLL PARAM
#define ROLL_ANGLE_REF 0.0f
#define ALI_KP 2.5f
// AUTO PITCH PARAM
#define PITCH_ANGLE_REF -20.0f
#define ELE_KP 3.0f
// AUTO YAW RATE PARAM
#define YAW_RATE_REF 0.0f
#define RUD_KP 0.4f
class Control
{
public:
    Control();
    void Initialize();                                                         // set motor output pin & initialize ESC
    void MainControl(Sbus *sbus, Sensor *sensor);                              // control servo motors
    void MotorControl(Sbus *sbus, Barometer *brm, float altitude);             // control thrust
    void MotorShutdown();                                                      // shutdown motor when no sbus input
    void DataMonitor(bool ifCheck) const;                                      // check ctl data
    void DataSDCardOutput(SDCardModule *sdc, File &file, const float &CurSec); // output to sd card
    void ActiveAutoYaw(bool ifActive);                                         // for mode change(region)

private:
    Servo servoObj[SERVO_INDEX];
    uint8_t servoOutputPin[SERVO_INDEX];
    uint16_t servoOutput[SERVO_INDEX];

    Servo ESCObj[ESC_INDEX];
    uint8_t ESCOutputPin[ESC_INDEX];

private:
    // left aileron control range: 0 - 180, flat: 90 
    int leftAileronAngle;
    // right aileron control range: 0 - 180, flat: 90 
    int rightAileronAngle;
    // elevator control range: 0 - 180, flat: 90 
    int elevatorAngle;
    // left+right throttle control range: 352-1696, max power: 1696 
    uint16_t thrust[2];
    // side-force Plate control CH4
    uint16_t sideForcePlate;
    // rudder control range: 0 - 180, flat: 90 
    uint16_t rudderAngle;

    // auto roll,pitch checker
    bool autoRoll;
    bool autoPitch;
    bool autoTakeoffYaw;

    // ros receive
    bool IfRosTrue;
    float ROSaltitude;
    float fixedAltitude;
    // auto thrust param
    bool idle;
    bool takeoff;
    bool takeoffInit;
    bool cruise;
    float currentTime;
    float takeoffTime;
    float takeoffPressure;
    float prevPressure;
    float pressure_diff;

    // auto altitude
    float altitudeRef;
    int pressureFixCount;
    float pressureFixSum;
};

#endif
