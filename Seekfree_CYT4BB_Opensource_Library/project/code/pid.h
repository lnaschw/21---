/*********************************************************************************************************************
 * Name:pid.h
 *  
 * Created on:
 * Author:
********************************************************************************************************************/

#ifndef _PID_H_
#define _PID_H_

typedef struct {
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float integral; // 积分项
    float derivative;
    float last_error; // 上一次误差
    float output;   // 输出值
    float error;
} PIDController;

// 前馈式 PID 控制器
// 与普通 PID 的区别：增加了 kff 前馈系数和 feedforward 前馈项
// output = Kp*error + Ki*integral + Kd*derivative + Kff*feedforward_value
// 前馈项直接根据目标值预判输出，不等误差产生就提前动作
typedef struct {
    float kp;            // 比例系数
    float ki;            // 积分系数
    float kd;            // 微分系数
    float kff;           // 前馈系数 —— 目标值到输出的预估映射
    float integral;      // 积分项
    float derivative;    // 微分项
    float last_error;    // 上一次误差
    float output;        // 最终输出
    float error;         // 当前误差
    float feedforward;   // 前馈分量（只读，调试用）
} FeedforwardPID;


// 普通 PID
void  PID_Init(PIDController *pid, float kp, float ki, float kd);
float PID_Update(PIDController *pid, float setpoint, float measured_value, float max_integral);

// 前馈式 PID
void  FeedforwardPID_Init(FeedforwardPID *pid, float kp, float ki, float kd, float kff);
float FeedforwardPID_Update(FeedforwardPID *pid, float setpoint, float measured_value,
                            float feedforward_value, float max_integral);

#endif /* CODE_ATTITUDE_CONTROL_H_ */
