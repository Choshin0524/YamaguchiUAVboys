#include "GravityEst.h"
#include "Arduino.h"
#include "YUBMath.h"

GravityEst::GravityEst()
{
}

void GravityEst::GravityEstRead(IMU& imu)
{
    imu.AX -= imu.AXave;
    imu.AY -= imu.AYave;
    imu.AZ = imu.AZ - imu.AZave + 1.0;
    imu.P = imu.Psen - imu.Pave;
    imu.Q = imu.Qsen - imu.Qave;
    imu.R = imu.Rsen - imu.Rave;
    for (i = 5; i > 0; i--)
    {
        Ix[i] = Ix[i - 1];
        Iy[i] = Iy[i - 1];
        Iz[i] = Iz[i - 1];
    }

    Ix[0] = imu.GetAX();
    Iy[0] = imu.GetAY();
    Iz[0] = imu.GetAZ();
    Ox = (Ix[5] + Ix[4] + Ix[3] + Ix[2] + Ix[1] + Ix[0]) / 6.0;
    Oy = (Iy[5] + Iy[4] + Iy[3] + Iy[2] + Iy[1] + Iy[0]) / 6.0;
    Oz = (Iz[5] + Iz[4] + Iz[3] + Iz[2] + Iz[1] + Iz[0]) / 6.0;

    imu.AX = Ox;
    imu.AY = Oy;
    imu.AZ = Oz;

    ar[0] = 
        imu.GetAX() * (sq(qest_[0]) + sq(qest_[1]) - sq(qest_[2]) - sq(qest_[3])) + 2 * 
        imu.GetAY() * (qest_[1] * qest_[2] - qest_[0] * qest_[3]) + 2 * 
        imu.GetAZ() * (qest_[1] * qest_[3] + qest_[0] * qest_[2]);

    ar[1] = 2 * 
        imu.GetAX() * (qest_[1] * qest_[2] + qest_[0] * qest_[3]) + 
        imu.GetAY() * (sq(qest_[0]) - sq(qest_[1]) + sq(qest_[2]) - sq(qest_[3])) + 2 * 
        imu.GetAZ() * (qest_[2] * qest_[3] - qest_[0] * qest_[1]);

    q_caret[0] = qest_[0] + 0.5 * (-qest_[1] * imu.GetP() - qest_[2] * imu.GetQ() - qest_[3] * imu.GetR()) * delta_t;
    q_caret[1] = qest_[1] + 0.5 * (qest_[0]  * imu.GetP() - qest_[3] * imu.GetQ() + qest_[2] * imu.GetR()) * delta_t;
    q_caret[2] = qest_[2] + 0.5 * (qest_[3]  * imu.GetP() + qest_[0] * imu.GetQ() - qest_[1] * imu.GetR()) * delta_t;
    q_caret[3] = qest_[3] + 0.5 * (-qest_[2] * imu.GetP() + qest_[1] * imu.GetQ() + qest_[0] * imu.GetR()) * delta_t;

    ag_caret[0] = g * q_caret[1] * q_caret[3] - g * q_caret[0] * q_caret[2] + g * q_caret[1] * q_caret[3] - g * q_caret[0] * q_caret[2];
    ag_caret[1] = g * q_caret[2] * q_caret[3] + g * q_caret[2] * q_caret[3] + g * q_caret[0] * q_caret[1] + g * q_caret[0] * q_caret[1];
    ag_caret[2] = g * q_caret[3] * q_caret[3] - g * q_caret[2] * q_caret[2] - g * q_caret[1] * q_caret[1] + g * q_caret[0] * q_caret[0];


    float sqAXAYAZ = sq(imu.GetAX()) + sq(imu.GetAY()) + sq(imu.GetAZ());
    C[0] = imu.GetAX() * sq(g) / sqAXAYAZ;
    C[1] = imu.GetAY() * sq(g) / sqAXAYAZ;
    C[2] = imu.GetAZ() * sq(g) / sqAXAYAZ

    for (i = 0; i < 3; i++)
        ag_caretS[i] = ag_caret[i] * (sq(C[0]) + sq(C[1]) + sq(C[2])) / (C[0] * ag_caret[0] + C[1] * ag_caret[1] + C[2] * ag_caret[2]);

    for (i = 0; i < 3; i++)
        a_C[i] = ag_caretS[i] - C[i];

    rr = g * g * (sqAXAYAZ - g * g) / sqAXAYAZ;

    u = sqrt(rr / (a_C[0] * a_C[0] + a_C[1] * a_C[1] + a_C[2] * a_C[2]));

    for (i = 0; i < 3; i++)
        r[i] = u * (ag_caretS[i] - C[i]);

    for (i = 0; i < 3; i++)
        ag[i] = C[i] + r[i];

    if (sqAXAYAZ - g * g < 0)
    {
        ag[0] = AX;
        ag[1] = AY;
        ag[2] = AZ;
    }
    a_m = 9.81 * sqrt(sq(ar[0]) + sq(ar[1]));
}
