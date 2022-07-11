#ifndef Control_h
#define Control_h
#include "Arduino.h"
#include "Sbus.h"
#include "IMU.h"
#include "Madgwick.h"
#include "Macros.h"

class Control
{
public:
    Control();
    void MainControl(Sbus& sbus, IMU& imu, Madgwick& mdg, int* CHANNEL);

public:


    float roll_ref = 0.0;
    float pitch_ref = 25.0;
    float H_ref = 3.0;
    float pitch_ref_DI;
    float V_ref = 5.0;
    float radius_ref = 6.0;

    int flag1 = 0;
    int flag2 = 0;
    int flag3 = 0;
    int ch2_correction, ch4_correction, ch5_correction;

    float u_theta;
    float u_V;
    float D;
    float T;

    float roll_integral = 0.0;
    float pitch_integral = 0.0;
    float H_integral = 0.0;
    float pitch_integral_DI = 0.0;
    float V_integral = 0.0;
    float radius_integral = 0.0;
    float sideslip_integral = 0.0;
    float yaw_rate_integral = 0.0;

    float ut;
    float K_t = 0.0;

    float yaw_rate0;

    float ail_deg;
    float ele_deg;
    float rud_deg;
    float sff_deg;
    float flap_deg;

    int servo[7];


};

#endif
