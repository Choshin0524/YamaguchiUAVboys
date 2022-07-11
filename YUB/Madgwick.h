#ifndef Madgwick_h
#define Madgwick_h
#include "Arduino.h"
#include "IMU.h"
#include "Macros.h"

class Madgwick
{
public:
    Madgwick();
    void MadgwickRead(IMU &imu);

public:
    float norm;
    float AXn, AYn, AZn;
    float fg[3] = {0, 0, 0};
    float Jg[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
    float grad[4] = {0, 0, 0, 0};
    float qeps[4] = {1, 0, 0, 0};
    float qomega[4] = {1, 0, 0, 0};
    float beta = sqrt(3.0f / 4.0f) * 3.14159265358979 * (5.0 / 180.0);
    float qest[4] = {1, 0, 0, 0};
    float qest_[4] = {1, 0, 0, 0};
    float roll, pitch, yaw;
    float yaw_tmp0 = 0.0;
    float yaw_tmp1 = 0.0;
    float yaw_unrap;
    int yaw_cnt = 0;
    float yaw_rate;

public:
    float Ox = 0.0;
    float Oy = 0.0;
    float Oz = 0.0;

    float Ix[6] = {0, 0, 0, 0, 0, 0};
    float Iy[6] = {0, 0, 0, 0, 0, 0};
    float Iz[6] = {0, 0, 0, 0, 0, 0};

    float a_m;
    float g = 1.0;
    float q_caret[4];
    float ag_caret[3];
    float C[3];
    float ag_caretS[3];
    float a_C[3];
    float rr;
    float u;
    float r[3];
    float ag[3];

    float ar[3];
    float I[6] = {0, 0, 0, 0, 0, 0};
    float O[6] = {0, 0, 0, 0, 0, 0};
    float a[6] = {-0.1254306, 0.8811301, -2.5452529, 3.8060181, -2.9754221, 1};
    float b[6] = {0.0012826, 0.0064129, 0.0128258, 0.0128258, 0.0064129, 0.0012826};
    float radius;
    float V_caret;
    float radius0 = 15.0;
};

#endif
