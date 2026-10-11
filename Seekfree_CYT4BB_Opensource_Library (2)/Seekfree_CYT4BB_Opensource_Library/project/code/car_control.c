/*********************************************************************************************************************
 * Name:car_control.c
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "kaerman.h"
#include "quaternion.h"
#include "car_control.h"
#include "my_encoder.h"
#include "ztjx.h"
#define GYRO_SCALE 14.3f

#define MAX_DUTY            (25)                                               // 最大 MAX_DUTY% 占空比
#define DIR_R1              (P09_1)
#define PWM_R1              (TCPWM_CH24_P09_0)
#define DIR_L1              (P05_1)
#define PWM_L1              (TCPWM_CH09_P05_0)

#define DIR_R2              (P10_3)
#define PWM_R2              (TCPWM_CH30_P10_2)
#define DIR_L2              (P05_3)
#define PWM_L2              (TCPWM_CH11_P05_2)

PIDController qianjin;
PIDController zuoyou;
PIDController xuanzhuan;
PIDController rv;
PIDController pv;
PIDController yv;
PIDController xv;
PIDController yyv;
PIDController back_x;
PIDController back_y;

int flag_get=0;
float erro_xx=0;
float erro_yy=0;
float erro_xx_1=0;
float erro_yy_1=0;
float car_qianjin= 0;
float car_zuoyou = 0;
float car_xuanzhuan= 0;
float car_xx=0;
float car_yy=0;
float yy_count=0;
float xx_count=0;
float need_back_x=0;
float  need_back_y=0;

float speed1=0;
float speed2=0;
float speed3=0;
float speed4=0;

float rcc_roll=0;
float rcc_pitch=0;
float rcc_yaw=0;

void car_Init(float kp1,float ki1,float kd1,float kp2,float ki2, float kd2,float kp3,float ki3,float kd3){
    
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    PID_Init(&qianjin,kp1,ki1,kd1);
    PID_Init(&zuoyou,kp2,ki2,kd2);
    PID_Init(&xuanzhuan,kp3,ki3,kd3);
    
    
    PID_Init(&rv,0.0,0.0,0.0);
    PID_Init(&pv,0.0,0.0,0.0);
    //PID_Init(&yv,0.0,0.0,0.0);
    
   // PID_Init(&xv,1.0,0.0,0.1);
    //PID_Init(&yyv,0.2,0.0,0.02);
    
    PID_Init(&xv,0.5,0.5,0.05);
    PID_Init(&yyv,0.5,0.5,0.05);
    
    PID_Init(&back_x,0.08,0.0,0.01);
    PID_Init(&back_y,0.08,0.0,0.01);
}
void motor_control(int8 lq,int8 rq,int8 lh,int8 rh){
  if(lq >=0)
  {
      if(lq>MAX_DUTY)
        lq=MAX_DUTY;
     gpio_set_level(DIR_L1, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_L1, lq * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(lq < 0)
   {    
     if((-lq)>MAX_DUTY)
        lq=-MAX_DUTY;
     gpio_set_level(DIR_L1, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_L1, (-lq) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  
  ////////
  if(rq >=0)
  {     
    if(rq>MAX_DUTY)
        rq=MAX_DUTY;
     gpio_set_level(DIR_R1, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_R1, rq * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(rq < 0)
   {    
     if((-rq)>MAX_DUTY)
        rq=-MAX_DUTY;
     gpio_set_level(DIR_R1, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_R1, (-rq) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  
  ////////
  if(lh >=0)
  {
    if(lh>MAX_DUTY)
        lh=MAX_DUTY;
     gpio_set_level(DIR_L2, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_L2, lh * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(lh < 0)
   {
     if((-lh)>MAX_DUTY)
        lh=-MAX_DUTY;
     gpio_set_level(DIR_L2, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_L2, (-lh) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  
  ////////
  if(rh >=0)
  {
    if(rh>MAX_DUTY)
        rh=MAX_DUTY;
     gpio_set_level(DIR_R2, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_R2, rh * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(rh < 0)
   {
     if((-rh)>MAX_DUTY)
        rh=-MAX_DUTY;
     gpio_set_level(DIR_R2, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_R2, (-rh) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
}

void car_pid(void)
{
  car_xuanzhuan = PID_Update(&xuanzhuan,0.0,s_angle.agz,10.0);
  car_qianjin = PID_Update(&qianjin,0.0,imu660rc_pitch,10);
  car_zuoyou= -PID_Update(&zuoyou,0.0,imu660rc_roll-5,10); 
  //printf("%f\n",imu660rc_roll);
  /*if (car_xuanzhuan>=10) car_xuanzhuan=10;
  if (car_xuanzhuan<=-10) car_xuanzhuan=-10;
  if (car_qianjin>=10) car_qianjin=10;
  if (car_qianjin<=-10) car_qianjin=-10;
  
  if (car_zuoyou>=10) car_zuoyou=15;
  if (car_zuoyou<=-10) car_zuoyou=-15;*/
  //printf("%f,%f,%f\n",s_angle.agz,imu660rc_pitch,imu660rc_roll-5);
}
void car_pid1 (void)
{  
  if(flag_get>=1&&flag_get<=7){
   
   need_back_x=0.0f;
   need_back_y=0.0f;
   //xx_count=0;
   //yy_count=0;
   erro_xx_1=erro_xx-94;
   erro_yy_1=erro_yy-60;
   car_xx = -PID_Update(&xv,0.0,erro_xx_1,10);
   car_yy = PID_Update(&yyv,0.0,erro_yy_1,10);
  //printf("%f\n",car_yy);
   //printf("%f,%f\n",erro_xx,erro_yy);
  }
  if(flag_get==-1)
  {
    need_back_x=PID_Update(&back_x,0.0,xx_count,10);
    need_back_y=PID_Update(&back_y,0.0,yy_count,10);
    
    if(need_back_x>=10)
    {
    need_back_x=10;
    }
    if(need_back_x<=-10)
    {
    need_back_x=-10;
    }
    
    if(need_back_y>=10)
    {
    need_back_y=10;
    }
    if(need_back_y<=-10)
    {
    need_back_y=-10;
    }
  }
}

void car_star(void){
  float go = 0;  //10
  //float yaw_speed;
  //car_xuanzhuan = PID_Update(&xuanzhuan,0.0,s_angle.agz+4.5f,10.0);
  //car_qianjin = PID_Update(&qianjin,0.0,s_angle.agy,10);
  //car_zuoyou= PID_Update(&zuoyou,0.0,s_angle.agx,10);
  //printf("%f,%f\n",car_qianjin,car_zuoyou);
  //yaw_speed = PID_Update(&yv,car_xuanzhuan,imu660rb_gyro_z_l,10.0);
  //printf("%f,%f,%f,%f\n",speed1,speed2,speed3,speed4);
  //printf("%f,%f\n",car_xx,car_yy);
  
  speed1 = PID_Update(&p1,go+car_xuanzhuan-car_qianjin-car_zuoyou-car_xx-need_back_x+car_yy+need_back_y ,encoder_data_dir[0]/500,10.0);
  speed2 = PID_Update(&p2,go+car_xuanzhuan-car_qianjin+car_zuoyou+car_xx+need_back_x+car_yy+need_back_y,encoder_data_dir[1]/500,10.0);
  speed3 = PID_Update(&p3,go-car_xuanzhuan-car_qianjin+car_zuoyou+car_xx+need_back_x+car_yy+need_back_y ,-encoder_data_dir[2]/500,10.0);
  speed4 = PID_Update(&p4,go-car_xuanzhuan-car_qianjin-car_zuoyou-car_xx-need_back_x+car_yy+need_back_y ,-encoder_data_dir[3]/500,10.0);
  
  /*speed1 = PID_Update(&p1,car_zuoyou,encoder_data_dir[0]/500,10.0);
  speed2 = PID_Update(&p2,-car_zuoyou,encoder_data_dir[1]/500,10.0);
  speed3 = PID_Update(&p3,-car_zuoyou,-encoder_data_dir[2]/500,10.0);
  speed4 = PID_Update(&p4,car_zuoyou ,-encoder_data_dir[3]/500,10.0);*/
  //printf("%f,%f,%f,%f\n",speed1,speed2,speed3,speed4);
  if(speed1 > 20.0f)  speed1=20.0f;
  if(speed2 > 20.0f)  speed2=20.0f;
  if(speed3 > 20.0f)  speed3=20.0f;
  if(speed4 > 20.0f)  speed4=20.0f;
  
  if(speed1 < -20.0f)  speed1=-20.0f;
  if(speed2 < -20.0f)  speed2=-20.0f;
  if(speed3 < -20.0f)  speed3=-20.0f;
  if(speed4 < -20.0f)  speed4=-20.0f;
 
  printf("%f,%f,%f,%f\n",speed1,speed2,speed3,speed4);
  
  //printf("%f,%f,%f\n",car_xuanzhuan,car_qianjin,car_zuoyou);
  //printf("%d ,%d ,%d ,%d\n",encoder_data_dir[0] ,encoder_data_dir[1] ,encoder_data_dir[2] ,encoder_data_dir[3]);
  
    int sped1=(int)speed1;
    int sped2=(int)speed2;
    int sped3=(int)speed3;
    int sped4=(int)speed4;
  
   motor_control(sped1,sped4,sped3,sped2);
   //motor_control(0,0,0,10);
   //motor_control(0,0,10,0);
                  
   //motor_control(0,0,0,20);
}