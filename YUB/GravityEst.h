#ifndef GravityEst_h
#define GravityEst_h
#include "Arduino.h"
#include "IMU.h"

//磁気センサー

class GravityEst
{
public:
    GravityEst();
    void GravityEstRead(IMU& imu);
private:
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
};

#endif