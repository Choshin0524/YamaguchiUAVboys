#include "Madgwick.h"
#include "Arduino.h"

Madgwick::Madgwick()
{
}

void Madgwick::MadgwickRead(IMU& imu)
{
    imu.AX -= imu.GetAXave(); 
    imu.AY -= imu.GetAYave();
    imu.AZ = imu.GetAZ() - imu.GetAZave() + 1.0;
    imu.P = imu.GetPsen() - imu.GetPave();
    imu.Q = imu.GetQsen() - imu.GetQave();
    imu.R = imu.GetRsen() - imu.GetRave();
    for (int i = 5; i > 0; i--)
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
    C[2] = imu.GetAZ() * sq(g) / sqAXAYAZ;

    for (int i = 0; i < 3; i++)
        ag_caretS[i] = ag_caret[i] * (sq(C[0]) + sq(C[1]) + sq(C[2])) / (C[0] * ag_caret[0] + C[1] * ag_caret[1] + C[2] * ag_caret[2]);

    for (int i = 0; i < 3; i++)
        a_C[i] = ag_caretS[i] - C[i];

    rr = g * g * (sqAXAYAZ - g * g) / sqAXAYAZ;

    u = sqrt(rr / (a_C[0] * a_C[0] + a_C[1] * a_C[1] + a_C[2] * a_C[2]));

    for (int i = 0; i < 3; i++)
        r[i] = u * (ag_caretS[i] - C[i]);

    for (int i = 0; i < 3; i++)
        ag[i] = C[i] + r[i];

    if (sqAXAYAZ - g * g < 0)
    {
        ag[0] = imu.GetAX();
        ag[1] = imu.GetAY();
        ag[2] = imu.GetAZ();
    }
    a_m = 9.81 * sqrt(sq(ar[0]) + sq(ar[1]));


    norm = sqrt(sq(ag[0]) + sq(ag[1]) + sq(ag[2]));
    AXn = ag[0] / norm;
    AYn = ag[1] / norm;
    AZn = ag[2] / norm;

    fg[0] = 2 * (qest_[1] * qest_[3] - qest_[0] * qest_[2]) - AXn;
    fg[1] = 2 * (qest_[0] * qest_[1] + qest_[2] * qest_[3]) - AYn;
    fg[2] = 2 * (0.5 - qest_[1] * qest_[1] - qest_[2] * qest_[2]) - AZn;

    Jg[0][0] = -2 * qest_[2];
    Jg[0][1] = 2 * qest_[3];
    Jg[0][2] = -2 * qest_[0];
    Jg[0][3] = 2 * qest_[1];
    Jg[1][0] = 2 * qest_[1];
    Jg[1][1] = 2 * qest_[0];
    Jg[1][2] = 2 * qest_[3];
    Jg[1][3] = 2 * qest_[2];
    Jg[2][0] = 0;
    Jg[2][1] = -4 * qest_[1];
    Jg[2][2] = -4 * qest_[2];
    Jg[2][3] = 0;

    grad[0] = Jg[0][0] * fg[0] + Jg[1][0] * fg[1] + Jg[2][0] * fg[2];
    grad[1] = Jg[0][1] * fg[0] + Jg[1][1] * fg[1] + Jg[2][1] * fg[2];
    grad[2] = Jg[0][2] * fg[0] + Jg[1][2] * fg[1] + Jg[2][2] * fg[2];
    grad[3] = Jg[0][3] * fg[0] + Jg[1][3] * fg[1] + Jg[2][3] * fg[2];

    norm = sqrt(sq(grad[0]) + sq(grad[1]) + sq(grad[2]) + sq(grad[3]));
    for (int i = 0; i < 4; i++)
        qeps[i] = grad[i] / norm;

    qomega[0] = 0.5 * (-qest_[1] * imu.GetP() - qest_[2] * imu.GetQ() - qest_[3] * imu.GetR());
    qomega[1] = 0.5 * (qest_[0] * imu.GetP() + qest_[2] * imu.GetR() - qest_[3] * imu.GetQ());
    qomega[2] = 0.5 * (qest_[0] * imu.GetQ() - qest_[1] * imu.GetR() + qest_[3] * imu.GetP());
    qomega[3] = 0.5 * (qest_[0] * imu.GetR() + qest_[1] * imu.GetQ() - qest_[2] * imu.GetP());

    for (int i = 0; i < 4; i++)
        qest[i] = qomega[i] - beta * qeps[i];

    for (int i = 0; i < 4; i++)
        qest[i] = qest_[i] + qest[i] * delta_t;

    norm = sqrt(qest[0] * qest[0] + qest[1] * qest[1] + qest[2] * qest[2] + qest[3] * qest[3]);
    for (int i = 0; i < 4; i++)
        qest[i] /= norm;
    for (int i = 0; i < 4; i++)
        qest_[i] = qest[i];

    roll = (180.0 / PI) * atan((2.0 * qest[2] * qest[3] + 2.0 * qest[0] * qest[1]) / (1.0 - 2.0 * qest[1] * qest[1] - 2.0 * qest[2] * qest[2]));
    pitch = -(180.0 / PI) * asin(2.0 * qest[0] * qest[2] - 2.0 * qest[1] * qest[3]) + 6.0;
    yaw = -(180.0 / PI) * atan2(2.0 * qest[1] * qest[2] + 2.0 * qest[0] * qest[3], 1.0 - 2.0 * qest[2] * qest[2] - 2.0 * qest[3] * qest[3]);

    yaw_rate = -(180.0 / PI) * ((imu.GetR() * cos((PI / 180.0) * roll) + imu.GetQ() * sin((PI / 180.0) * roll)) / (cos((PI / 180.0) * pitch)));
    radius = a_m / (yaw_rate * (PI / 180.0) * yaw_rate * (PI / 180.0));
    if (!((radius > 3.0) && (radius < 15.0)))
        radius = radius0;

    for (int i = 5; i > 0; i--)
        I[i] = I[i - 1];
    for (int i = 5; i > 0; i--)
        O[i] = O[i - 1];
    I[0] = radius;
    O[0] = ((b[0] * I[5] + b[1] * I[4] + b[2] * I[3] + 
                b[3] * I[2] + b[4] * I[1] + b[5] * I[0]) - 
                (a[0] * O[5] + a[1] * O[4] + a[2] * O[3] + a[3] * O[2] + a[4] * O[1])) / a[5];
    radius = O[0];
    V_caret = (PI / 180.0) * radius * yaw_rate;
}