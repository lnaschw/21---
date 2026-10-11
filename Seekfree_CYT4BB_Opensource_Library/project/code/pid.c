/*********************************************************************************************************************
 * Name:pid.c
 *      姿态控制
 * Created on:
 * Author:
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "matrix.h"


void PID_Init(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->last_error = 0.0;
    pid->output = 0.0;
    pid->error = 0.0;
}

// 更新PID控制器          结构体              目标值               实际值                 积分限幅        
float PID_Update(PIDController *pid, float set_value, float real_value,float max_integral)
{
  
    pid->error = set_value - real_value;//计算新误差

    pid->integral += pid->error;// 积分项累加

    // 积分抗饱和
    if (pid->integral > max_integral)
     {
         pid->integral = max_integral;
     }
     else if (pid->integral < -max_integral)
     {
         pid->integral = -max_integral;
     }

    // 计算微分项
    pid->derivative = (pid->error - pid->last_error);

    pid->last_error = pid->error;//将旧error存起来
    
    // 计算 PID 输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
    
    return pid->output;
}

/*float PID_Update(PIDController *pid, float mubiaozhi,float shijizhi1,float max)
{
    pid->error=mubiaozh-shijizh;
    
    pid->integral+=pid->error;
    
    if (pid->integral > max_integral)
     {
         pid->integral = max_integral;
     }
     else if (pid->integral < -max_integral)
     {
         pid->integral = -max_integral;
     }
      
    pid->derivative = (pid->error - pid->last_error);

    pid->last_error = pid->error;
    
    pid->output=pid->kp*pid->error+pid->ki*pid->integral+pid->kd*pid->derivative:
      
      returnpid->output;

}*/


