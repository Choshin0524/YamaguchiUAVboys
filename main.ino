#include<Wire.h>
#include<SD.h>
#include<math.h>
#include<WiFi.h>
#include<WiFiUdp.h>
#include "UAVmacros.h"
#include "Sbus.h"
HardwareSerial UART2(2);

// EPS32, 9軸センサ, 動作確認用サーボ
#define MPU6050 0x68
#define Airspeed3 0x0C //x
#define Airspeed2 0x0B //y
#define Airspeed1 0x0A //z

#define PI 3.14159265358979

//IMU()
uint8_t Buf[25];

int16_t A_x_sen,A_y_sen,A_z_sen;
double AX,AY,AZ;
double AXave,AYave,AZave;

int16_t G_x_sen,G_y_sen,G_z_sen;
double P,Q,R;
double Psen,Qsen,Rsen;
double Pave,Qave,Rave;

//Airspeed_sensor
int16_t V_I2C;
double V_sen[12];
double VX,VY,VZ,V;
double attack;
double sideslip;
int z=0;

//PSD()
int adc_key_in;
double e;
double E;
double k = 2.4;
double H = 0.0;
double H_;
double H_0 = 0.0;

//gravity_est()
double delta_t = 0.020;

double Ox = 0.0;
double Oy = 0.0;
double Oz = 0.0;

double Ix[6] = {0,0,0,0,0,0};
double Iy[6] = {0,0,0,0,0,0};
double Iz[6] = {0,0,0,0,0,0};

double g = 1;
double q_caret[4];
double ag_caret[3];
double C[3];
double ag_caretS[3];
double a_C[3];
double rr;
double u;
double r[3];
double ag[3];

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
double a_m;
double I[6] = {0,0,0,0,0,0};
double O[6] = {0,0,0,0,0,0};
double a[6] = {-0.1254306,0.8811301,-2.5452529,3.8060181,-2.9754221,1};
double b[6] = {0.0012826,0.0064129,0.0128258,0.0128258,0.0064129,0.0012826};
double radius;
double radius0 = 15.0;
double sideslip0 = 0.0;
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

//SD_micro()
File dataFile;

unsigned long Time = 0;

//setup()
int i;

int IN[7];
int CHANNEL[7];

//loop()
unsigned long dt;
unsigned long time1 = 0;

//wifi
byte x1H = 255;
const char ssid[] = "yamatake wifi"; // SSID
const char pass[] = "yamatake wifi";  // password
const IPAddress ip(192, 168, 4, 2);       // IPアドレス(ゲートウェイも兼ねる)
const IPAddress subnet(255, 255, 255, 0); // サブネットマスク
const char * udpAddress = "192.168.4.2";
WiFiUDP UDP1;

void I2Cread(uint8_t Address,uint8_t Register,uint8_t Byte,uint8_t* Data){
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  Wire.requestFrom(Address,Byte);
  uint8_t index = 0;
  while (Wire.available()) Data[index++] = Wire.read();
}

void I2Cwrite(uint8_t Address,uint8_t Register,uint8_t Data){
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void IMU(void){
  I2Cwrite(MPU6050,0x1C,0x18);
  I2Cwrite(MPU6050,0x1D,0x06);
  I2Cwrite(MPU6050,0x1B,0x10);
  I2Cwrite(MPU6050,0x1A,0x06);
  I2Cread(MPU6050,0x3B,14,Buf);
  
  A_x_sen = (Buf[0]<<8|Buf[1]);
  A_y_sen = (Buf[2]<<8|Buf[3]);
  A_z_sen = Buf[4]<<8|Buf[5];
  AY = -(double)(A_x_sen*0.488*0.001);
  AX = (double)(A_y_sen*0.488*0.001);
  AZ = (double)(A_z_sen*0.488*0.001);
  G_x_sen = (Buf[8]<<8|Buf[9]); 
  G_y_sen = (Buf[10]<<8|Buf[11]); 
  G_z_sen = Buf[12]<<8|Buf[13]; 
  Qsen = -(double)((PI/180.0)*G_x_sen*0.03048);
  Psen = (double)((PI/180.0)*G_y_sen*0.03048);
  Rsen = (double)((PI/180.0)*G_z_sen*0.03048);
}

void Airspeed_sensor(void){
  I2Cread(Airspeed3,0x03,1,&Buf[1]);
  I2Cread(Airspeed3,0x04,1,&Buf[2]);
  I2Cread(Airspeed2,0x03,1,&Buf[3]);
  I2Cread(Airspeed2,0x04,1,&Buf[4]);
  I2Cread(Airspeed1,0x03,1,&Buf[5]);
  I2Cread(Airspeed1,0x04,1,&Buf[6]);
  /*I2Cread(Airspeed3,0x0D,1,&Buf[7]);
  I2Cread(Airspeed3,0x0E,1,&Buf[8]);
  I2Cread(Airspeed2,0x0D,1,&Buf[9]);
  I2Cread(Airspeed2,0x0E,1,&Buf[10]);
  I2Cread(Airspeed1,0x0D,1,&Buf[11]);
  I2Cread(Airspeed1,0x0E,1,&Buf[12]);
  I2Cread(Airspeed3,0x0F,1,&Buf[13]);
  I2Cread(Airspeed3,0x10,1,&Buf[14]); 
  I2Cread(Airspeed2,0x0F,1,&Buf[15]);
  I2Cread(Airspeed2,0x10,1,&Buf[16]);
  I2Cread(Airspeed1,0x0F,1,&Buf[17]);
  I2Cread(Airspeed1,0x10,1,&Buf[18]);
  I2Cread(Airspeed3,0x11,1,&Buf[19]);
  I2Cread(Airspeed3,0x12,1,&Buf[20]);
  I2Cread(Airspeed2,0x11,1,&Buf[21]);
  I2Cread(Airspeed2,0x12,1,&Buf[22]);
  I2Cread(Airspeed1,0x11,1,&Buf[23]);
  I2Cread(Airspeed1,0x12,1,&Buf[24]);*/

  V_I2C = ((int16_t)(Buf[1]&0xFF)<<8)|(int16_t)(Buf[2]&0xFF);
  V_sen[0] = (double)(V_I2C)/100.0;
  V_I2C = ((int16_t)(Buf[3]&0xFF)<<8)|(int16_t)(Buf[4]&0xFF);
  V_sen[1] = (double)(V_I2C)/100.0;
  V_I2C = ((int16_t)(Buf[5]&0xFF)<<8)|(int16_t)(Buf[6]&0xFF);
  V_sen[2] = (double)(V_I2C)/100.0;

  /*V_I2C = ((int16_t)(Buf[7]&0xFF)<<8)|(int16_t)(Buf[8]&0xFF);
  V_sen[3] = (double)(V_I2C);
  V_I2C= ((int16_t)(Buf[9]&0xFF)<<8)|(int16_t)(Buf[10]&0xFF);
  V_sen[4] = (double)(V_I2C);
  V_I2C= ((int16_t)(Buf[11]&0xFF)<<8)|(int16_t)(Buf[12]&0xFF);
  V_sen[5] = (double)(V_I2C);
  
  V_I2C= ((int16_t)(Buf[13]&0xFF)<<8)|(int16_t)(Buf[14]&0xFF);
  V_sen[6] = (double)(V_I2C)/100.0;
  V_I2C= ((int16_t)(Buf[15]&0xFF)<<8)|(int16_t)(Buf[16]&0xFF);
  V_sen[7] = (double)(V_I2C)/100.0;
  V_I2C= ((int16_t)(Buf[17]&0xFF)<<8)|(int16_t)(Buf[18]&0xFF);
  V_sen[8] = (double)(V_I2C)/100.0;

  V_I2C= ((int16_t)(Buf[19]&0xFF)<<8)|(int16_t)(Buf[20]&0xFF);
  V_sen[9] = (double)(V_I2C)/100.0;
  V_I2C= ((int16_t)(Buf[21]&0xFF)<<8)|(int16_t)(Buf[22]&0xFF);
  V_sen[10] = (double)(V_I2C)/100.0;
  V_I2C= ((int16_t)(Buf[23]&0xFF)<<8)|(int16_t)(Buf[24]&0xFF);
  V_sen[11] = (double)(V_I2C)/100.0;*/
  
  VX = V_sen[1]*cos(45.0*(PI/180.0))+V_sen[2]*cos(45.0*(PI/180.0));
  VY = -V_sen[2]*sin(45.0*(PI/180.0))+V_sen[1]*sin(45.0*(PI/180.0));
  VZ = V_sen[0];
  
  V = sqrt(VX*VX+VY*VY+VZ*VZ);
  attack = atan2(VZ,VX)*(180.0/PI)+5.0;
  sideslip = atan2(VY,sqrt(VX*VX+VZ*VZ))*(180.0/PI);
}

void PSD(void){
  E = 0.0;
  for(i=0;i<20;i++){
    adc_key_in = analogRead(35);
    e = adc_key_in*3.3/4096.0;
    E = E+e;
  }
  E = E/20.0;
  H = 8.1892*pow(E,4)-64.14*pow(E,3)+188.5*pow(E,2)-247.73*E+124.94;
  //H_ = k/(3.4/k*E-1)*(-0.2455*pow(E,3)+1.6018*pow(E,2)-3.4907*E+3.5574);
  if(!((H>1.0)&&(H<5.5))) H = H_0;
  H_0 = H;
}

void gravity_est(void){
  AX = AX-AXave;
  AY = AY-AYave;
  AZ = AZ-AZave+1.0;
  P = Psen-Pave;
  Q = Qsen-Qave;
  R = Rsen-Rave;
  
  for(i=5;i>0;i--){
    Ix[i] = Ix[i-1];
    Iy[i] = Iy[i-1];
    Iz[i] = Iz[i-1];
  }

  Ix[0] = AX;
  Iy[0] = AY;
  Iz[0] = AZ;

  Ox = (Ix[5]+Ix[4]+Ix[3]+Ix[2]+Ix[1]+Ix[0])/6.0;
  Oy = (Iy[5]+Iy[4]+Iy[3]+Iy[2]+Iy[1]+Iy[0])/6.0;
  Oz = (Iz[5]+Iz[4]+Iz[3]+Iz[2]+Iz[1]+Iz[0])/6.0;

  AX = Ox;
  AY = Oy;
  AZ = Oz;

  ar[0] = AX*(qest_[0]*qest_[0]+qest_[1]*qest_[1]-qest_[2]*qest_[2]-qest_[3]*qest_[3])+2*AY*(qest_[1]*qest_[2]-qest_[0]*qest_[3])+2*AZ*(qest_[1]*qest_[3]+qest_[0]*qest_[2]);
  ar[1] = 2*AX*(qest_[1]*qest_[2]+qest_[0]*qest_[3])+AY*(qest_[0]*qest_[0]-qest_[1]*qest_[1]+qest_[2]*qest_[2]-qest_[3]*qest_[3])+2*AZ*(qest_[2]*qest_[3]-qest_[0]*qest_[1]);
  
  q_caret[0] = qest_[0]+0.5*(-qest_[1]*P-qest_[2]*Q-qest_[3]*R)*delta_t;
  q_caret[1] = qest_[1]+0.5*(qest_[0]*P-qest_[3]*Q+qest_[2]*R)*delta_t;
  q_caret[2] = qest_[2]+0.5*(qest_[3]*P+qest_[0]*Q-qest_[1]*R)*delta_t;
  q_caret[3] = qest_[3]+0.5*(-qest_[2]*P+qest_[1]*Q+qest_[0]*R)*delta_t;
  
  ag_caret[0] = g*q_caret[1]*q_caret[3]-g*q_caret[0]*q_caret[2]+g*q_caret[1]*q_caret[3]-g*q_caret[0]*q_caret[2];
  ag_caret[1] = g*q_caret[2]*q_caret[3]+g*q_caret[2]*q_caret[3]+g*q_caret[0]*q_caret[1]+g*q_caret[0]*q_caret[1];
  ag_caret[2] = g*q_caret[3]*q_caret[3]-g*q_caret[2]*q_caret[2]-g*q_caret[1]*q_caret[1]+g*q_caret[0]*q_caret[0];

  C[0] = AX*(g*g)/(AX*AX+AY*AY+AZ*AZ);
  C[1] = AY*(g*g)/(AX*AX+AY*AY+AZ*AZ);
  C[2] = AZ*(g*g)/(AX*AX+AY*AY+AZ*AZ);

  for(i=0;i<3;i++) ag_caretS[i] = ag_caret[i]*(C[0]*C[0]+C[1]*C[1]+C[2]*C[2])/(C[0]*ag_caret[0]+C[1]*ag_caret[1]+C[2]*ag_caret[2]);

  for(i=0;i<3;i++) a_C[i] = ag_caretS[i]-C[i];

  rr = g*g*(AX*AX+AY*AY+AZ*AZ-g*g)/(AX*AX+AY*AY+AZ*AZ);
  
  u = sqrt(rr/(a_C[0]*a_C[0]+a_C[1]*a_C[1]+a_C[2]*a_C[2]));

  for(i=0;i<3;i++) r[i] = u*(ag_caretS[i]-C[i]);

  for(i=0;i<3;i++) ag[i] = C[i]+r[i];

  if(AX*AX+AY*AY+AZ*AZ-g*g<0){
    ag[0] = AX;
    ag[1] = AY;
    ag[2] = AZ;
  }
  a_m = 9.81*sqrt(ar[0]*ar[0]+ar[1]*ar[1]);
}

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
  /*
  yaw_tmp0 = yaw_tmp1;
  yaw_tmp1 = yaw;
  yaw_unrap = yaw_tmp1-yaw_tmp0;
  if(yaw_unrap<-300.0) yaw_cnt++;
  if(yaw_unrap>300.0) yaw_cnt--;
  yaw = yaw+yaw_cnt*360.0;
  */
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
    
    ut = -(PI/180.0)*0.1*yaw_rate;
    ch[3] = (1-ut)*(ch[3]-offset[3])-0.5*(ch[1]-offset[1])+offset[3];//右
    ch[6] = (1+ut)*(ch[6]-offset[6])+0.5*(ch[1]-offset[1])+offset[6];//左
    //ch[3] = (1-K_t)*(ch[3]-offset[3])-0.5*(ch[1]-offset[1])+offset[3];//右
    //ch[6] = (1+K_t)*(ch[6]-offset[6])+0.5*(ch[1]-offset[1])+offset[6];//左 
    roll_integral = roll_integral+(roll-roll_ref)*delta_t;
    ch[1] = Kp_a*(roll-roll_ref)+Kd_a*P+Ki_a*roll_integral+offset[1];
    radius0 = radius;
    sideslip0 = sideslip;
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

void SD_micro(){
  if(ch[11]>1500){
    dataFile = SD.open("/twingerZ.txt", FILE_APPEND);
    
    Time += dt;
    
    if(dataFile){
      dataFile.print(dt);dataFile.print(" , ");
      dataFile.print(Time); dataFile.print(" , ");
      dataFile.print(AX); dataFile.print(" , ");
      dataFile.print(AY); dataFile.print(" , ");
      dataFile.print(AZ); dataFile.print(" , ");
      dataFile.print(P); dataFile.print(" , ");
      dataFile.print(Q); dataFile.print(" , ");
      dataFile.print(R); dataFile.print(" , ");
      dataFile.print(roll); dataFile.print(" , ");
      dataFile.print(pitch); dataFile.print(" , ");
      dataFile.print(yaw); dataFile.print(" , ");
      dataFile.print(yaw_rate); dataFile.print(" , ");
      dataFile.print(a_m); dataFile.print(" , ");
      dataFile.print(V_sen[0]); dataFile.print(" , ");
      dataFile.print(V_sen[1]); dataFile.print(" , ");
      dataFile.print(V_sen[2]); dataFile.print(" , ");
      /*dataFile.print(V_sen[3]); dataFile.print(" , ");
      dataFile.print(V_sen[4]); dataFile.print(" , ");
      dataFile.print(V_sen[5]); dataFile.print(" , ");
      dataFile.print(V_sen[6]); dataFile.print(" , ");
      dataFile.print(V_sen[7]); dataFile.print(" , ");
      dataFile.print(V_sen[8]); dataFile.print(" , ");
      dataFile.print(V_sen[9]); dataFile.print(" , ");
      dataFile.print(V_sen[10]); dataFile.print(" , ");
      dataFile.print(V_sen[11]); dataFile.print(" , ");*/
      dataFile.print(H); dataFile.print(" , ");
      dataFile.print(radius); dataFile.print(" , ");
      //dataFile.print(V_caret); dataFile.print(" , ");
      dataFile.print(ail_deg); dataFile.print(" , ");
      dataFile.print(ele_deg); dataFile.print(" , ");
      dataFile.print(rud_deg); dataFile.print(" , ");
      dataFile.print(sff_deg); dataFile.print(" , ");
      dataFile.print(ch[10]); dataFile.print(" , ");
      //dataFile.print(ch[12]); dataFile.print(" , ");
      dataFile.print(ch[3]); dataFile.print(" , ");
      dataFile.print(ch[6]); dataFile.print(" , ");
      dataFile.print(ut); dataFile.print(" , ");
      dataFile.print(VX); dataFile.print(" , ");
      dataFile.print(VY); dataFile.print(" , ");
      dataFile.print(VZ); dataFile.print(" , ");
      dataFile.print(V); dataFile.print(" , ");
      dataFile.print(attack); dataFile.print(" , ");
      dataFile.print(sideslip); dataFile.print(" , ");
      if(x1H != 255){ //受信データに255が大量に含まれているため(wifiエラー?)
        dataFile.print(x1H); dataFile.print(" , ");
      }
      dataFile.println();
      dataFile.close();
    }
  }
}


void Display()
{
  //Serial.print(dt1); Serial.print(",,, ");
  Serial.print(Time); Serial.print(",,, ");
  Serial.print(roll); Serial.print(", ");
  Serial.print(pitch); Serial.print(", ");
  Serial.print(yaw); Serial.print(", ");
  //Serial.print(yaw_cnt); Serial.print(", ");
  //Serial.print(ch[1]); Serial.print(", ");
  //Serial.print(ch[2]); Serial.print(", ");
  //Serial.print(ch[3]); Serial.print(", ");
  //Serial.print(ch[4]); Serial.print(", ");
  //Serial.print(ch[5]); Serial.print(", ");
  //Serial.print(ch[6]); Serial.print(", ");
  //Serial.print(ch[7]); Serial.print(", ");
  //Serial.print(ch[8]); Serial.print(", ");
  //Serial.print(ch[9]); Serial.print(", ");
  //Serial.print(H); Serial.print(", ");
  //Serial.print(ch[11]); Serial.print(", ");
  //Serial.print(ch[12]); Serial.print(", ");
  //Serial.print(yaw_rate); Serial.print(", ");
  Serial.print(pitch_integral); Serial.print(", ");
  //Serial.print(ele_deg); Serial.print(", ");
  //Serial.print(H); Serial.print(", ");
  //Serial.print(H_); Serial.print(", ");
  Serial.println();
}

void wifi()
{
  if (!UDP1.parsePacket()) { 
    x1H = UDP1.read(); //wifiデータ受信(1バイト)
    }
}

void setup(void){
  Serial.begin(115200);
  Sbus sbus1;

  pinMode(12, OUTPUT);
  SD.begin(5,SPI,24000000,"/sd");

  IN[1-1] = 17;
  IN[2-1] = 33;
  IN[3-1] = 27;
  IN[4-1] = 25;
  IN[5-1] = 14;
  IN[6-1] = 26;
  IN[7-1] = 13;
  for(i=0;i<7;i++){
    CHANNEL[i] = i;
    ledcSetup(CHANNEL[i],50,10);
    ledcAttachPin(IN[i],CHANNEL[i]);
  }

    
  Wire.begin();
  
  I2Cwrite(MPU6050,27,0x10);
  I2Cwrite(MPU6050,28,0x18);
  I2Cwrite(MPU6050,0x37,0x02);
  I2Cwrite(MPU6050,0x6B,0x00);
  
  for (i=0;i<1000;i++){
    IMU();
    AXave += AX/1000;
    AYave += AY/1000;
    AZave += AZ/1000;
    Pave += Psen/1000;
    Qave += Qsen/1000;
    Rave += Rsen/1000;
  }
  　　
  WiFi.softAP(ssid, pass);           // SSIDとパスの設定
  delay(100);                        // 追記：このdelayを入れないと失敗する場合がある
  WiFi.softAPConfig(ip, ip, subnet); // IPアドレス、ゲートウェイ、サブネットマスクの設定
  Serial.print("AP IP address: ");
  IPAddress myIP = WiFi.softAPIP();
  Serial.println(myIP);
  Serial.println("Starting UDP");
  UDP1.begin(111);  // UDP通信の開始(引数はポート番号)
    
  delay(100);
}

void loop(void){

  sbus1.SbusRead(); // フタバ工業：sbus規格デジタル信号を読み込む
  IMU();　// 9-軸センサを読み取る（6軸のみ使用）
  PSD(); // 高度制御 / 高度計
  gravity_est(); // 姿勢角？の計算
  madgwick(); // マグフィルター（姿勢角を出すための計算）
  control(); // ＰＩＤ制御（モーター、エレベーターやラダー）
  wifi(); // wifi（処理落ちするかも）
  SD_micro(); // フライトデータ書き込み用
  Display(); // デバッグ用

  // if(z == 4)
  // {
  //   Airspeed_sensor();
  // }

  // z++;
  // if(z==5) z=0;

  while(millis()<time1);
  dt = (unsigned long)(millis()-time1+20);
  time1 = (unsigned long)(millis()+20);
}
