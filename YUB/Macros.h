// EPS32, 9軸センサ, 動作確認用サーボ
#define MPU6050 0x68
#define Airspeed3 0x0C //x
#define Airspeed2 0x0B //y
#define Airspeed1 0x0A //z

#define PI 3.14159265358979

#define Kp_a 20.0
#define Kd_a 10.0
#define Ki_a 5.0
#define Kp_e 28.0 //28 18
#define Kd_e 7.0 //7 5
#define Ki_e 1.0 //7 1
#define Kp_H 10.0
#define Kd_H 0.0
#define Ki_H 0.0
#define Kp_e_DI 10.0
#define Kd_e_DI 0.0
#define Ki_e_DI 0.0
#define Kp_V 10.0
#define Kd_V 0.0
#define Ki_V 0.0
#define Kp_R 25.0 //25 50
#define Kd_R 7.0
#define Ki_R 15.0 //15 40
#define Kp_B 20.0 //10.0
#define Kd_B 4.0 //2.0
#define Ki_B 10.0 //8.0
#define Kp_psirate 10.0
#define Kd_psirate 0.0
#define Ki_psirate 0.0