/*********************************************************************************************************************
 * Name:flycontrol.c
 *      飞控
 * Created on:
 * Author:  田深深深深深深
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "kaerman.h"
#include "quaternion.h"


#define MAX_DUTY                (30)                    // 最大 测试 占空比 30%
#define MIN_DUTY                (10)                    // 最小 测试 占空比 10%
                                                                     
                                                                     
#define PMW_CHANNEL_1           (TCPWM_CH19_P08_0)      // pwm输出通道1                                 
#define PMW_CHANNEL_2           (TCPWM_CH20_P08_1)      // pwm输出通道2                             
#define PMW_CHANNEL_3           (TCPWM_CH21_P08_2)      // pwm输出通道3                                                                   
#define PMW_CHANNEL_4           (TCPWM_CH22_P08_3)      // pwm输出通道4


PIDController hight;
PIDController Roll;
PIDController Yaw;
PIDController Pitch;
PIDController XX;
PIDController YY;
PIDController PMW_X;
PIDController PMW_Y;
float hhhh;
float yyyy;
float rrrr;
float pppp;
float offest_x;
float offest_y;
float pmw_xx;
float pmw_yy;
float target_xx;
float target_yy;
float highter;//高度
volatile float PPXX;
float PPYY;

void pid_Init_all(float hight_p,float hight_i,float hight_d,float Roll_p,float Roll_i,float Roll_d,float Yaw_p,float Yaw_i,float Yaw_d,float Pitch_p,float Pitch_i,float Pitch_d,float XX_p,
                  float XX_i,float XX_d,float YY_p,float YY_i,float YY_d,float PMW_X_p,float PMW_X_i,float PMW_X_d,float PMW_Y_p,float PMW_Y_i,float PMW_Y_d)
{
  PID_Init(&hight,hight_p,hight_i,hight_d);
  PID_Init(&Roll,Roll_p,Roll_i,Roll_d);
  PID_Init(&Yaw,Yaw_p,Yaw_i,Yaw_d);
  PID_Init(&Pitch,Pitch_p,Pitch_i,Pitch_d);
  PID_Init(&XX,XX_p,XX_i,XX_d);
  PID_Init(&YY,YY_p,YY_i,YY_d);
  PID_Init(&PMW_X,PMW_X_p,PMW_X_i,PMW_X_d);
  PID_Init(&PMW_Y,PMW_Y_p,PMW_Y_i,PMW_Y_d);
  
  pwm_init(PMW_CHANNEL_1, 400, 0);                    // 初始化pwm引脚功能  默认400hz  0%占空比
  pwm_init(PMW_CHANNEL_2, 400, 0);
  pwm_init(PMW_CHANNEL_3, 400, 0);
  pwm_init(PMW_CHANNEL_4, 400, 0);

}
 
void fly_star (void)
{   
    target_xx = PID_Update(&XX,94.0,offest_x,100);//摄像头
    target_yy = PID_Update(&YY,60.0,offest_y,100);
    
    hhhh= PID_Update(&hight,800.0,dl1b_distance_mm,1200.0);//TOF 
    
    pmw_xx=PID_Update(&PMW_X,0+target_xx,PPXX,100);//光流
    pmw_yy=PID_Update(&PMW_Y,0+target_yy,PPYY,100);
    
    yyyy= PID_Update(&Yaw,0,euler_angle.yaw,100);//电机要的
    rrrr= PID_Update(&Roll,0,euler_angle.roll,100); 
    pppp= PID_Update(&Pitch,0,euler_angle.pitch,100);
    
    yyyy=yyyy*300;
    int32 yyy = (int32)-yyyy;
    
    int32 hhh= (int32)hhhh;
    
    int32 rrr = (int32)-rrrr;
    
    int32 ppp = (int32)-pppp;
   // printf("%d,\n",yyy);
    
    uint32 M1=6000+hhh+rrr/2+ppp/2;
    uint32 M2=6000+hhh+rrr/2-ppp/2;
    uint32 M3=6000+hhh-rrr/2+ppp/2;
    uint32 M4=6000+hhh-rrr/2-ppp/2;
   //printf("hhh:%d\n",hhh);
    //printf("ppp:%d\n",ppp);
    //printf("rrr:%d\n",rrr);
    if(M1>8000) M1=8000;
    if(M2>8000) M2=8000;
    if(M3>8000) M3=8000;
    if(M4>8000) M4=8000;
    
    printf("M1:%d,M2%dM3,%dM4,%d\n",M1,M2,M3,M4);
    
    pwm_set_duty(PMW_CHANNEL_1,M1 );//+rrr +ppp -yyy                      // 输出占空比数值  输出值 = 占空比设置值 + 1ms固定占空比
    pwm_set_duty(PMW_CHANNEL_2,M2);//+rrr -ppp +yyy
    pwm_set_duty(PMW_CHANNEL_3,M3);//-rrr +ppp +yyy
    pwm_set_duty(PMW_CHANNEL_4,M4);//-rrr -ppp-yyy
}
