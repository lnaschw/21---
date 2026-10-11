/*********************************************************************************************************************
 * Name:attitude_control.c
 *      姿态控制
 * Created on:
 * Author:
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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

// 更新PID控制器          结构体              目标值               实际值                 积分限幅             机械零点
float PID_Update(PIDController *pid, float setpoint, float measured_value,float max_integral,float inherent_error)
{

    pid->last_error = pid->error;//将旧error存起来

    pid->error = setpoint - (measured_value - inherent_error);//计算新误差

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

    // 计算 PID 输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;

    return pid->output;
}
//一。前后平衡PID控制框架
//读取外部数据：1电机行进速度 2车模前后倾角 3前后角速度
float read_speed()
{
    // 这里替换为实际读取电机速度的代码
    float speed = single_speed;
    return speed;
}
float read_angle()
{
    //实际读取车模前后倾角的代码
    float angle = final_pitch;
    return angle;
}
float read_angular_velocity()
{
    //实际读取车模前后角速度的代码
    float angular_velocity = change_gz;
    return angular_velocity;
}
// 限制底轮PWM输出范围
float clamp_pwm(float pwm, float min_pwm, float max_pwm)
{
   if (pwm < min_pwm) return min_pwm;
   if (pwm > max_pwm) return max_pwm;
   return pwm;
}

//二。左右平衡PID控制框架
//读取外部数据：1采集两个飞轮速度 2车模左右倾角 3左右角速度
float double_speed()
{  //两个飞轮速度处理

    float double_speed = (float)((-motor_value.receive_left_speed_data) + motor_value.receive_right_speed_data)/2;
    return double_speed;
}

float leftright_angle()
{  //计算车模左右倾角
    float leftright_angle =  final_roll ;
    return leftright_angle;
}

float leftright_angle_velocity()
{  //计算车模左右角速度
    float leftright_angle_velocity =  change_gy ;
    return leftright_angle_velocity;
}
// 限制飞轮PWM输出范围
float clamp_pwm_2(float pwm, float min_pwm, float max_pwm)
{
   if (pwm < min_pwm) return min_pwm;
   if (pwm > max_pwm) return max_pwm;
   return pwm;
}


float complementaryFilter(float accel_data, float gyro_data, float prev_output, float alpha, float dt)
{
    // 计算互补滤波输出
    float output = alpha * (prev_output + gyro_data * dt) + (1 - alpha) * accel_data;
    prev_output = output;

    return output;
}

float really_euler_angle(float acc, float gyro , float parameter)
{
     float euler_angle = parameter * gyro + (1-parameter) * acc ;
     return  euler_angle;
}

void LowPassFilter_Init(LowPassFilter *filter, float cutoff_freq, float sample_time)
{
    // 计算滤波系数 alpha
    float rc = 1.0f / (2 * M_PI * cutoff_freq);
    filter->alpha = sample_time / (rc + sample_time);

    // 限制 alpha 范围 (避免异常值)
    if (filter->alpha < 0.0f) filter->alpha = 0.0f;
    else if (filter->alpha > 1.0f) filter->alpha = 1.0f;

    // 初始状态清零
    filter->filtered_value = 0.0f;
}
float LowPassFilter_Update(LowPassFilter *filter, float raw_input)
{
    // 一阶低通滤波公式
    filter->filtered_value = filter->alpha * raw_input +
                             (1.0f - filter->alpha) * filter->filtered_value;
    return filter->filtered_value;
}

void imu_data_get(void)
{
      imu660rb_get_gyro();
      imu660rb_get_acc();

    change_gx = imu660rb_gyro_transition (imu660rb_gyro_x) ;
    change_gy = imu660rb_gyro_transition (imu660rb_gyro_y) ;
    change_gz = imu660rb_gyro_transition (imu660rb_gyro_z) ;

    change_ax =   imu660rb_acc_transition (imu660rb_acc_x);
    change_ay =   imu660rb_acc_transition (imu660rb_acc_y);
    change_az =   imu660rb_acc_transition (imu660rb_acc_z);

    int actual_car_gx = change_gy ;
    int actual_car_gy = change_gz ;
    int actual_car_gz = change_gx ;

    double actual_car_ax = change_ay  ;
    double actual_car_ay = change_az  ;
    double actual_car_az = change_ax  ;

    g_yaw  = actual_car_gz * 0.008 + final_yaw;
    g_pitch = actual_car_gy * 0.008 + final_pitch;
    g_roll = actual_car_gx * 0.008 + final_roll;

    a_yaw = 0 ;
    a_pitch = -(atan2(actual_car_ax,actual_car_az)/3.1415927*180)  ;
    a_roll = atan2(actual_car_ay,actual_car_az)/3.1415927*180  ;

    final_yaw = g_yaw;
    final_pitch = really_euler_angle(a_pitch, g_pitch , 0.96666);
    final_roll = really_euler_angle(a_roll, g_roll , 0.96666);

}

void Blind_turn(void)
{

    target_turn = target_turn_3;
}

//以下是新增的增量式PID代码
void PID_Init_2(IncrementalPID *pid, float Kp, float Ki, float Kd, float target, float max_out, float min_out)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->max_output = max_out;
    pid->min_output = min_out;
    pid->output = 0;
}

float PID_Calculate(IncrementalPID *pid, float feedback, float dt)
{
    if (dt <= 0) return pid->output;  // 防止除以0或负时间

    float error =  feedback + pid->target  ;

    // 计算增量
    float delta_output = pid->Kp * (error - pid->last_error) +
                         pid->Ki * error * dt +
                         pid->Kd * (error - 2 * pid->last_error + pid->prev_error) / dt;

    // 更新误差历史
    pid->prev_error = pid->last_error;
    pid->last_error = error;

    // 计算输出
    pid->output += delta_output;

    // 输出限幅
    if (pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }
    else if (pid->output < pid->min_output)
    {
        pid->output = pid->min_output;
    }

    return pid->output;
}

float filterGyroRate(float current_rate, float alpha)
{
    static float filtered_rate = 0.0f; // 静态变量保存滤波结果

    // 首次调用时用当前值初始化
    if (filtered_rate == 0.0f && current_rate != 0.0f) {
        filtered_rate = current_rate;
    }

    // 一阶IIR滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    filtered_rate = alpha * current_rate + (1.0f - alpha) * filtered_rate;

    return filtered_rate;
}



