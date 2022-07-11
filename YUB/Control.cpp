#include "Control.h"
#include "Arduino.h"

Control::Control()
{}

void Control::MainControl(Sbus& sbus, IMU& imu, Madgwick& mdg, int* CHANNEL)
{
    if (sbus.GetCh(8) >= 1400)
    { //ロール自動

        if (sbus.GetCh(8) >= 2000)
        {

            if (flag1 == 0)
            {
                ch2_correction = sbus.GetCh(2) - sbus.offset[2];
                flag1 = 1;
            }

            //ピッチ角一定目標値追従制御
            pitch_integral = pitch_integral + (mdg.pitch - pitch_ref) * delta_t;
            sbus.ch[2] = -Kp_e * (mdg.pitch - pitch_ref) - Kd_e * imu.GetQ() - Ki_e * pitch_integral + ch2_correction + sbus.offset[2];
            // ch[2] = 75.0+sbus.offset[2]; //75=5[deg]
        }
        else
        {
            pitch_integral = 0.0;
            flag1 = 0;
        }

        if (sbus.ch[10] <= 2000)
        {
            if (flag2 == 0)
            {
                radius_integral = 0.0;
                sideslip_integral = 0.0;
                yaw_rate0 = mdg.yaw_rate;
                flag2 = 1;
            }
            else
            {
                flag2 = 0;
            }
            ch4_correction = sbus.GetCh(4) - sbus.offset[4];
            ch5_correction = sbus.GetCh(5) - sbus.offset[5];

            radius_integral = radius_integral + (mdg.radius - radius_ref) * delta_t;

            //左旋回
            if (yaw_rate0 < 0)
            {
                sbus.ch[4] = -Kp_R * (mdg.radius - radius_ref) - Ki_R * radius_integral - Kd_R * (mdg.radius - mdg.radius0) + ch4_correction + sbus.offset[4];
                sbus.ch[5] = Kp_R * (mdg.radius - radius_ref) + Ki_R * radius_integral + Kd_R * (mdg.radius - mdg.radius0) + ch5_correction + sbus.offset[5];
            }
            //右旋回
            else
            {
                sbus.ch[4] = Kp_R * (mdg.radius - radius_ref) + Ki_R * radius_integral + Kd_R * (mdg.radius - mdg.radius0) + ch4_correction + sbus.offset[4];
                sbus.ch[5] = -Kp_R * (mdg.radius - radius_ref) - Ki_R * radius_integral - Kd_R * (mdg.radius - mdg.radius0) + ch5_correction + sbus.offset[5];
            }
        }

        roll_integral = roll_integral + (mdg.roll - roll_ref) * delta_t;
        sbus.ch[1] = Kp_a * (mdg.roll - roll_ref) + Kd_a * imu.GetP() + Ki_a * roll_integral + sbus.offset[1];
        mdg.radius0 = mdg.radius;
    }
    else
        roll_integral = 0.0;

    if (sbus.GetCh(9) >= 1400)
        sbus.ch[7] = sbus.offset[7] + 400;

    if (sbus.GetCh(3) > 2100)
        sbus.ch[3] = 2100;
    if (sbus.GetCh(6) > 2100)
        sbus.ch[6] = 2100;

    ail_deg = (sbus.ch[1] - sbus.offset[1]) / 600.0 * 25.0;
    ele_deg = (sbus.ch[2] - sbus.offset[2]) / 600.0 * 40.0;
    sff_deg = (sbus.ch[4] - sbus.offset[4]) / 600.0 * 32.5;
    rud_deg = (sbus.ch[5] - sbus.offset[5]) / 600.0 * 30.0;
    flap_deg = (sbus.ch[7] - sbus.offset[7]) / 600.0 * 30.0;

    for (int i = 0; i < 7; i++)
    {
        servo[i] = (int)(sbus.ch[i + 1] * 1024 / 20000.0);
        if (servo[i] > 108)
            servo[i] = 108;
        if (servo[i] < 51)
            servo[i] = 51;
        ledcWrite(CHANNEL[i], servo[i]);
    }
}