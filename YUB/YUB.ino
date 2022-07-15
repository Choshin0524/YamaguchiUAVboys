#include <math.h>
#include "Sbus.h"
#include "IMU.h"
#include "Control.h"
#include "Madgwick.h"

// initialize
HardwareSerial UART2(2);
Sbus sbus(UART2);
IMU imu;
Madgwick mdg;
Control ctl;
unsigned long dt;
unsigned long time1 = 0;
// pinをCHANNELに割り当て
int IN[7] = {17, 33, 27, 25, 14, 26, 13};
int CHANNEL[7];

void setup(void)
{
  Serial.begin(115200);

  for (int i = 0; i < 7; i++)
  {
    CHANNEL[i] = i;
    ledcSetup(CHANNEL[i], 50, 10);
    ledcAttachPin(IN[i], CHANNEL[i]);
  }
}

void loop(void)
{
  sbus.SbusRead(UART2); // フタバ工業：sbus規格デジタル信号を読み込む
  imu.IMURead();        // 9-軸センサを読み取る（6軸のみ使用）
  mdg.MadgwickRead(imu);      // マグフィルター（姿勢角を出すための計算）
  ctl.MainControl(sbus, imu, mdg, CHANNEL); // ＰＩＤ制御（モーター、エレベーターやラダー）

  while (millis() < time1);
  dt = (unsigned long)(millis() - time1 + 20);
  time1 = (unsigned long)(millis() + 20);
}
