#include<SD.h>
#include<math.h>
#include "Macros.h"
#include "Sbus.h"
#include "IMU.h"
#include "PSD.h"
#include "YUBWifi.h"
#include "GravityEst.h"

//madgwick()
double norm;
double AXn,AYn,AZn;
double fg[3] = {0,0,0};
double Jg[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
double grad[4] = {0,0,0,0};
double qeps[4] = {1,0,0,0};
double qomega[4] = {1,0,0,0};
double beta = sqrt(3.0f/4.0f)*3.14159265358979*(5.0/180.0);
double qest[4] = {1,0,0,0};
double qest_[4] = {1,0,0,0};
double roll,pitch,yaw;
double yaw_tmp0 = 0.0;
double yaw_tmp1 = 0.0;
double yaw_unrap;
int yaw_cnt = 0;
double yaw_rate;

//control()
double ar[3];
double I[6] = {0,0,0,0,0,0};
double O[6] = {0,0,0,0,0,0};
double a[6] = {-0.1254306,0.8811301,-2.5452529,3.8060181,-2.9754221,1};
double b[6] = {0.0012826,0.0064129,0.0128258,0.0128258,0.0064129,0.0012826};
double radius;
double radius0 = 15.0;
double V_caret;

double roll_ref = 0.0;
double pitch_ref = 25.0;
double H_ref = 3.0;
double pitch_ref_DI;
double V_ref = 5.0;
double radius_ref = 6.0;
double sideslip_ref = -5.0;

int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int16_t ch2_correction,ch4_correction,ch5_correction;

double u_theta;
double u_V;
double D;
double T;

double roll_integral = 0.0;
double pitch_integral = 0.0;
double H_integral = 0.0;
double pitch_integral_DI = 0.0;
double V_integral = 0.0;
double radius_integral = 0.0;
double sideslip_integral = 0.0;
double yaw_rate_integral = 0.0;

double ut;
double K_t = 0.0;

double yaw_rate0;

double ail_deg;
double ele_deg;
double rud_deg;
double sff_deg;
double flap_deg;

int servo[7];
//setup()


//loop()
unsigned long dt;
unsigned long time1 = 0;

void madgwick(void){
  norm = sqrt(ag[0]*ag[0]+ag[1]*ag[1]+ag[2]*ag[2]);
  AXn = ag[0]/norm;
  AYn = ag[1]/norm;
  AZn = ag[2]/norm;

  fg[0] = 2*(qest_[1]*qest_[3]-qest_[0]*qest_[2])-AXn;
  fg[1] = 2*(qest_[0]*qest_[1]+qest_[2]*qest_[3])-AYn;
  fg[2] = 2*(0.5-qest_[1]*qest_[1]-qest_[2]*qest_[2])-AZn;
  
  Jg[0][0] = -2*qest_[2];
  Jg[0][1] = 2*qest_[3];
  Jg[0][2] = -2*qest_[0];
  Jg[0][3] = 2*qest_[1];
  Jg[1][0] = 2*qest_[1];
  Jg[1][1] = 2*qest_[0];
  Jg[1][2] = 2*qest_[3];
  Jg[1][3] = 2*qest_[2];
  Jg[2][0] = 0;
  Jg[2][1] = -4*qest_[1];
  Jg[2][2] = -4*qest_[2];
  Jg[2][3] = 0;
  
  grad[0] = Jg[0][0]*fg[0]+Jg[1][0]*fg[1]+Jg[2][0]*fg[2];
  grad[1] = Jg[0][1]*fg[0]+Jg[1][1]*fg[1]+Jg[2][1]*fg[2];
  grad[2] = Jg[0][2]*fg[0]+Jg[1][2]*fg[1]+Jg[2][2]*fg[2];
  grad[3] = Jg[0][3]*fg[0]+Jg[1][3]*fg[1]+Jg[2][3]*fg[2];
  
  norm = sqrt(grad[0]*grad[0]+grad[1]*grad[1]+grad[2]*grad[2]+grad[3]*grad[3]);
  for(i=0;i<4;i++) qeps[i] = grad[i]/norm;

  qomega[0] = 0.5*(-qest_[1]*P- qest_[2]*Q-qest_[3]*R);
  qomega[1] = 0.5*(qest_[0]*P+qest_[2]*R-qest_[3]*Q);
  qomega[2] = 0.5*(qest_[0]*Q-qest_[1]*R+qest_[3]*P);
  qomega[3] = 0.5*(qest_[0]*R+qest_[1]*Q-qest_[2]*P);
  
  for(i=0;i<4;i++) qest[i] = qomega[i]-beta*qeps[i];
  
  for(i=0;i<4;i++) qest[i] = qest_[i]+qest[i]*delta_t;

  norm = sqrt(qest[0]*qest[0]+qest[1]*qest[1]+qest[2]*qest[2]+qest[3]*qest[3]);
  for(i=0;i<4;i++) qest[i] /= norm;
  for(i=0;i<4;i++) qest_[i] = qest[i];
  
  roll  = (180.0/PI)*atan((2.0*qest[2]*qest[3]+2.0*qest[0]*qest[1])/(1.0-2.0*qest[1]*qest[1]-2.0*qest[2]*qest[2]));
  pitch = -(180.0/PI)*asin(2.0*qest[0]*qest[2]-2.0*qest[1]*qest[3])+6.0;
  yaw   = -(180.0/PI)*atan2(2.0*qest[1]*qest[2]+2.0*qest[0]*qest[3],1.0-2.0*qest[2]*qest[2]-2.0*qest[3]*qest[3]);

  yaw_rate = -(180.0/PI)*((R*cos((PI/180.0)*roll)+Q*sin((PI/180.0)*roll))/(cos((PI/180.0)*pitch)));
  radius = a_m/(yaw_rate*(PI/180.0)*yaw_rate*(PI/180.0));
  if(!((radius>3.0)&&(radius<15.0))) radius = radius0;
  

  for(i=5;i>0;i--) I[i] = I[i-1];
  for(i=5;i>0;i--) O[i] = O[i-1];
  I[0] = radius;
  O[0] = ((b[0]*I[5]+b[1]*I[4]+b[2]*I[3]+b[3]*I[2]+b[4]*I[1]+b[5]*I[0])-(a[0]*O[5]+a[1]*O[4]+a[2]*O[3]+a[3]*O[2]+a[4]*O[1]))/a[5];
  radius = O[0];
  V_caret = (PI/180.0)*radius*yaw_rate;
}

void control(void){
  if(ch[8]>=1400){ //ロール自動
    
    if(ch[8]>=2000){
    
      if(flag1==0){
        ch2_correction = ch[2]-offset[2];
        flag1 = 1;
      }
      
      //ピッチ角一定目標値追従制御
      pitch_integral = pitch_integral+(pitch-pitch_ref)*delta_t;
      ch[2] = - Kp_e*(pitch-pitch_ref)-Kd_e*Q-Ki_e*pitch_integral+ch2_correction+offset[2];
      //ch[2] = 75.0+offset[2]; //75=5[deg]    
     
    }
    else{
      pitch_integral = 0.0;
      flag1 = 0;
    }
   
    if(ch[10]<=2000){
      if(flag2==0){
        radius_integral = 0.0;
        sideslip_integral = 0.0;
        yaw_rate0 = yaw_rate;
        flag2 = 1;
      }
      else{
       flag2 = 0;
      }
      ch4_correction = ch[4]-offset[4];
      ch5_correction = ch[5]-offset[5];
      
      radius_integral = radius_integral+(radius-radius_ref)*delta_t;
      sideslip_integral = sideslip_integral+(sideslip-sideslip_ref)*delta_t;

      //左旋回
      if(yaw_rate0<0){
        ch[4] = -Kp_R*(radius-radius_ref)-Ki_R*radius_integral-Kd_R*(radius-radius0)+ch4_correction+offset[4];
        ch[5] = Kp_R*(radius-radius_ref)+Ki_R*radius_integral+Kd_R*(radius-radius0)+ch5_correction+offset[5];
        //ch[5] = Kp_B*(sideslip-sideslip_ref)+Ki_B*sideslip_integral+Kd_B*(sideslip-sideslip0)+ch5_correction+offset[5];
      }
      //右旋回
      else{
        ch[4] = Kp_R*(radius-radius_ref)+Ki_R*radius_integral+Kd_R*(radius-radius0)+ch4_correction+offset[4];
        ch[5] = -Kp_R*(radius-radius_ref)-Ki_R*radius_integral-Kd_R*(radius-radius0)+ch5_correction+offset[5];
        //ch[5] = -Kp_B*(sideslip-sideslip_ref)-Ki_B*sideslip_integral-Kd_B*(sideslip-sideslip0)+ch5_correction+offset[5];
      }
      
    }
    
    roll_integral = roll_integral+(roll-roll_ref)*delta_t;
    ch[1] = Kp_a*(roll-roll_ref)+Kd_a*P+Ki_a*roll_integral+offset[1];
    radius0 = radius;
  }
  else roll_integral = 0.0;
  
  if(ch[9]>=1400) ch[7] = offset[7]+400;

  if (ch[3] > 2100)ch[3] = 2100;
  if (ch[6] > 2100)ch[6] = 2100;
  
  ail_deg = (ch[1]-offset[1])/600.0*25.0;
  ele_deg = (ch[2]-offset[2])/600.0*40.0;
  sff_deg = (ch[4]-offset[4])/600.0*32.5;
  rud_deg = (ch[5]-offset[5])/600.0*30.0;
  flap_deg = (ch[7]-offset[7])/600.0*30.0;
  
  for(i=0;i<7;i++){
    servo[i] = (int)(ch[i+1]*1024/20000.0);
    if(servo[i]>108)servo[i] = 108;
    if(servo[i]<51)servo[i] = 51;
    ledcWrite(CHANNEL[i],servo[i]);
  }
}

void Display()
{
  Serial.print(Time); Serial.print(",,, ");
  Serial.print(roll); Serial.print(", ");
  Serial.print(pitch); Serial.print(", ");
  Serial.print(yaw); Serial.print(", ");
  Serial.print(pitch_integral); Serial.print(", ");
  Serial.println();
}

// initialize
HardwareSerial UART2(2);
Sbus sbus(UART2);
IMU imu;
PSD psd;
GravityEst gEst;

void setup(void){

  Serial.begin(115200);
  
  pinMode(12, OUTPUT);
  SD.begin(5,SPI,24000000,"/sd");

  // pinをCHANNELに割り当て
  int IN[7] = {17, 33, 27, 25, 14, 26, 13};
  int CHANNEL[7];
  for(int i = 0; i < 7; i++)
  {
    CHANNEL[i] = i;
    ledcSetup(CHANNEL[i], 50, 10);
    ledcAttachPin(IN[i], CHANNEL[i]);
  }
  YUBWifi yubWifi;
}

void loop(void){

  sbus.SbusRead(UART2); // フタバ工業：sbus規格デジタル信号を読み込む
  imu.IMURead(); // 9-軸センサを読み取る（6軸のみ使用）
  psd.PSDRead();
  gEst.GravityEstRead(imu); // 磁気の計算
  madgwick(); // マグフィルター（姿勢角を出すための計算）
  control(); // ＰＩＤ制御（モーター、エレベーターやラダー）
  yubWifi.WifiRead(); // wifi（処理落ちするかも）
  Display(); // デバッグ用

  while(millis()<time1);
  dt = (unsigned long)(millis()-time1+20);
  time1 = (unsigned long)(millis()+20);
}