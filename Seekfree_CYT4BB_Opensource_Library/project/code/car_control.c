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

#define MAX_DUTY            (25)                                               // ��� MAX_DUTY% ռ�ձ�
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

int flag_get=-1;
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
static int c_count=0;

// ---- menghe() 顺时针旋转一周控制变量 ----
static uint8_t  menghe_active = 0;          // 旋转模式标志位
static float    menghe_angle_sum = 0.0f;     // 累计顺时针旋转角度 (度)
static float    menghe_last_yaw = 0.0f;      // 上一次偏航角，用于计算增量
static uint16_t menghe_ramp_cnt = 0;         // 缓启动计数器

#define MENGHE_TARGET       360.0f           // 目标：旋转一周
#define MENGHE_BASE_SPEED   6.0f             // 基础旋转速度
#define MENGHE_RAMP_CYCLES  50               // 缓启动周期数 (50 * 10ms = 500ms)
#define MENGHE_DECEL_ANGLE  40.0f            // 剩余多少度时开始减速
#define MENGHE_MIN_SPEED    2.0f             // 最小旋转速度，防止堵转

void car_Init(float kp1,float ki1,float kd1,float kp2,float ki2, float kd2,float kp3,float ki3,float kd3){
    
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_R1, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_L1, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_R2, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_L2, 17000, 0);                                                 // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    
    PID_Init(&qianjin,kp1,ki1,kd1);
    PID_Init(&zuoyou,kp2,ki2,kd2);
    PID_Init(&xuanzhuan,kp3,ki3,kd3);
    
    
    PID_Init(&rv,0.0,0.0,0.0);
    PID_Init(&pv,0.0,0.0,0.0);
    //PID_Init(&yv,0.0,0.0,0.0);
    
   // PID_Init(&xv,1.0,0.0,0.1);
    //PID_Init(&yyv,0.2,0.0,0.02);
    
    PID_Init(&xv,0.5,0.0,0.05);
    PID_Init(&yyv,0.005,0.5,0.01);
    
    PID_Init(&back_x,0.08,0.0,0.01);
    PID_Init(&back_y,0.08,0.0,0.01);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数名          menghe
// 功能说明        控制麦轮小车顺时针方向旋转一周（360度）
// 参数说明        void
// 返回参数        void
// 使用示例        menghe();
// 备注           基于 IMU 偏航角反馈，通过 car_xuanzhuan 驱动四轮麦轮运动学旋转
//                 调用后自动完成一周旋转，旋转期间再次调用会被忽略
//                 旋转方向由 car_xuanzhuan 的符号决定（负值=顺时针），如需逆时针可修改符号
//-------------------------------------------------------------------------------------------------------------------
void menghe(void)
{
    if (menghe_active) return;          // 已在旋转中，忽略重复调用

    menghe_active = 1;
    menghe_angle_sum = 0.0f;
    menghe_ramp_cnt = 0;

    // 陀螺仪初始化 —— 重置偏航角归零，确保旋转角度从零开始累积
    kaerman_b_Init(&gyro_x, 2.0f, 2.0f, 80.0f, 80.0f, 0.002f, 0.002f);
    kaerman_b_Init(&gyro_y, 1.0f, 1.0f, 80.0f, 80.0f, 0.002f, 0.002f);
    kaerman_b_Init(&gyro_z, 1.0f, 1.0f, 80.0f, 80.0f, 0.002f, 0.002f);
    quaternion_Init(&sc, &s_angle, &s_shu, 1.2, 0.001, 0.002f);
    imu660rb_init();
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);

    menghe_last_yaw = s_angle.agz;      // 陀螺仪复位后记录起始偏航角（约0度）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名          menghe_is_done
// 功能说明        查询旋转是否完成
// 参数说明        void
// 返回参数        uint8_t: 1=旋转完成, 0=正在旋转
// 使用示例        if (menghe_is_done()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8_t menghe_is_done(void)
{
    return !menghe_active;
}

void motor_control(int8 lq,int8 rq,int8 lh,int8 rh){
  if(lq >=0)
  {
      if(lq>MAX_DUTY)
        lq=MAX_DUTY;
     gpio_set_level(DIR_L1, GPIO_HIGH);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_L1, lq * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  if(lq < 0)
   {    
     if((-lq)>MAX_DUTY)
        lq=-MAX_DUTY;
     gpio_set_level(DIR_L1, GPIO_LOW);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_L1, (-lq) * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  
  ////////
  if(rq >=0)
  {     
    if(rq>MAX_DUTY)
        rq=MAX_DUTY;
     gpio_set_level(DIR_R1, GPIO_HIGH);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_R1, rq * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  if(rq < 0)
   {    
     if((-rq)>MAX_DUTY)
        rq=-MAX_DUTY;
     gpio_set_level(DIR_R1, GPIO_LOW);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_R1, (-rq) * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  
  ////////
  if(lh >=0)
  {
    if(lh>MAX_DUTY)
        lh=MAX_DUTY;
     gpio_set_level(DIR_L2, GPIO_HIGH);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_L2, lh * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  if(lh < 0)
   {
     if((-lh)>MAX_DUTY)
        lh=-MAX_DUTY;
     gpio_set_level(DIR_L2, GPIO_LOW);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_L2, (-lh) * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  
  ////////
  if(rh >=0)
  {
    if(rh>MAX_DUTY)
        rh=MAX_DUTY;
     gpio_set_level(DIR_R2, GPIO_HIGH);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_R2, rh * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
  if(rh < 0)
   {
     if((-rh)>MAX_DUTY)
        rh=-MAX_DUTY;
     gpio_set_level(DIR_R2, GPIO_LOW);                                  // DIR����ߵ�ƽ
     pwm_set_duty(PWM_R2, (-rh) * (PWM_DUTY_MAX / 100));                  // ����ռ�ձ�
  }
}

void car_pid(void)
{
  if (menghe_active)
  {
      // ---- 顺时针旋转模式 ----
      // 计算本周期偏航角增量（10ms间隔）
      float delta = s_angle.agz - menghe_last_yaw;

      // 处理角度 ±180° 环绕
      if (delta > 180.0f)  delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;

      // 累加顺时针旋转角度（delta为负表示顺时针，取反后累加为正）
      menghe_angle_sum += (-delta);
      menghe_last_yaw = s_angle.agz;

      // 判断是否完成一周旋转
      if (menghe_angle_sum >= MENGHE_TARGET)
      {
          // 旋转完成，恢复正常偏航稳定
          menghe_active = 0;
          car_xuanzhuan = PID_Update(&xuanzhuan, 0.0, s_angle.agz, 10.0);
      }
      else
      {
          float remaining = MENGHE_TARGET - menghe_angle_sum;
          float speed = MENGHE_BASE_SPEED;

          // 缓启动：前 MENGHE_RAMP_CYCLES 个周期线性加速
          if (menghe_ramp_cnt < MENGHE_RAMP_CYCLES)
          {
              speed = MENGHE_BASE_SPEED * (float)(menghe_ramp_cnt + 1)
                      / (float)MENGHE_RAMP_CYCLES;
              menghe_ramp_cnt++;
          }

          // 缓停止：剩余角度小于阈值时线性减速
          if (remaining < MENGHE_DECEL_ANGLE)
          {
              float decel_speed = MENGHE_BASE_SPEED * (remaining / MENGHE_DECEL_ANGLE);
              if (decel_speed < speed) speed = decel_speed;
          }

          // 最小速度限制，防止静摩擦力导致堵转
          if (speed < MENGHE_MIN_SPEED) speed = MENGHE_MIN_SPEED;

          // 负值 = 顺时针方向（与实际麦轮安装方向相关，可调整符号）
          car_xuanzhuan = -speed;

          // 直接驱动麦轮旋转，绕过 car_star() 混控
          // 顺时针: L1正转, R1反转, L2正转, R2反转
          {
              int8 duty = (int8)(speed * 3.5f);       // 将速度映射到 PWM 占空比 (2→7%, 6→21%)
              if (duty > MAX_DUTY) duty = MAX_DUTY;
              motor_control(duty, -duty, duty, -duty);
              //             L1    R1    L2    R2
          }
      }
  }
  else
  {
      car_xuanzhuan = PID_Update(&xuanzhuan, 0.0, s_angle.agz, 10.0);
  }

  car_qianjin = -PID_Update(&qianjin,0.0,imu660rc_pitch -2.0,10);
  car_zuoyou= -PID_Update(&zuoyou,0.0,imu660rc_roll,10);
  //printf("%f\n",imu660rc_roll);
  /*if (car_xuanzhuan>=20) car_xuanzhuan=20;
  if (car_xuanzhuan<=-20) car_xuanzhuan=-20;*/
  if (car_qianjin>=10) car_qianjin=10;
  if (car_qianjin<=-10) car_qianjin=-10;
  if (car_zuoyou>=10) car_zuoyou=10;
  if (car_zuoyou<=-10) car_zuoyou=-10;
  //printf("%f\n",s_angle.agz);
}
void car_pid1 (void)
{  
  if(flag_get==1){
   
   need_back_x=0.0f;
   need_back_y=0.0f;
   //xx_count=0;
   //yy_count=0;
   erro_xx_1=erro_xx;
   erro_yy_1=erro_yy;
   car_xx = -PID_Update(&xv,0.0,erro_xx_1,10);
   car_yy = -PID_Update(&yyv,0.0,erro_yy_1,10);
  //printf("%f\n",car_yy);
   //printf("%f,%f\n",erro_xx,erro_yy);
  }
  if(flag_get==0)
  {
    need_back_x=xx_count;
    need_back_y=yy_count;
    
    if(need_back_x>=2)
    {
    need_back_x=2;
    }
    if(need_back_x<=-2)
    {
    need_back_x=-2;
    }
    
    if(need_back_y>=2)
    {
    need_back_y=2;
    }
    if(need_back_y<=-2)
    {
    need_back_y=-2;
    }
  }
}

void car_star(void){
  if (menghe_active) return;          // menghe 旋转期间跳过正常混控，由 car_pid() 直接驱动电机

  float go = 0;
  c_count++;
    if(c_count<=750)
      go=20.0;
   else
     go =0.0;
  //float yaw_speed;
  //car_xuanzhuan = PID_Update(&xuanzhuan,0.0,s_angle.agz+4.5f,10.0);
  //car_qianjin = PID_Update(&qianjin,0.0,s_angle.agy,10);
  //car_zuoyou= PID_Update(&zuoyou,0.0,s_angle.agx,10);
  //printf("%f\n",car_zuoyou);
  //yaw_speed = PID_Update(&yv,car_xuanzhuan,imu660rb_gyro_z_l,10.0);
  //printf("%f,%f,%f,%f\n",speed1,speed2,speed3,speed4);
  //printf("%f,%f\n",car_xx,car_yy);
  
  speed1 = PID_Update(&p1,go+car_xuanzhuan-car_qianjin+car_zuoyou-car_xx-need_back_x+car_yy+need_back_y ,encoder_data_dir[0]/500,10.0);
  speed2 = PID_Update(&p2,go+car_xuanzhuan-car_qianjin-car_zuoyou+car_xx+need_back_x+car_yy+need_back_y,encoder_data_dir[1]/500,10.0);
  speed3 = PID_Update(&p3,go-car_xuanzhuan-car_qianjin-car_zuoyou+car_xx+need_back_x+car_yy+need_back_y ,-encoder_data_dir[2]/500,10.0);
  speed4 = PID_Update(&p4,go-car_xuanzhuan-car_qianjin+car_zuoyou-car_xx-need_back_x+car_yy+need_back_y ,-encoder_data_dir[3]/500,10.0);
  
  /*speed1 = PID_Update(&p1,car_zuoyou,encoder_data_dir[0]/500,10.0);
  speed2 = PID_Update(&p2,-car_zuoyou,encoder_data_dir[1]/500,10.0);
  speed3 = PID_Update(&p3,-car_zuoyou,-encoder_data_dir[2]/500,10.0);
  speed4 = PID_Update(&p4,car_zuoyou ,-encoder_data_dir[3]/500,10.0);*/
  
  if(speed1 > 20.0f)  speed1=10.0f;
  if(speed1 > 20.0f)  speed1=10.0f;
  if(speed1 > 20.0f)  speed1=10.0f;
  if(speed1 > 20.0f)  speed1=10.0f;
  
  if(speed1 < -20.0f)  speed1=-10.0f;
  if(speed1 < -20.0f)  speed1=-10.0f;
  if(speed1 < -20.0f)  speed1=-10.0f;
  if(speed1 < -20.0f)  speed1=-10.0f;
 
  //printf("%f,%f,%f,%f\n",speed1,speed2,speed3,speed4);
  
  //printf("%f,%f,%f\n",car_xuanzhuan,yaw_speed,speed1);
  //printf("%d ,%d ,%d ,%d\n",encoder_data_dir[0] ,encoder_data_dir[1] ,encoder_data_dir[2] ,encoder_data_dir[3]);
  
    int sped1=(int)speed1;
    int sped2=(int)speed2;
    int sped3=(int)speed3;
    int sped4=(int)speed4;
  
   motor_control(sped1,sped4,sped3,sped2);
    //motor_control(0,0,sped3,0);
   //motor_control(0,0,0,10);
   //motor_control(0,0,10,0);
                  
   //motor_control(0,0,0,20);
}