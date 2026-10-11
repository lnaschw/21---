/*********************************************************************************************************************
 * Name:quaternion.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#ifndef _QUATERNION_H_
#define _QUATERNION_H_
typedef struct
{
    float kp;
    float ki;
    float qdt;
}my_qu;
typedef struct
{
    float agx;
    float agy;
    float agz;
}quat_angle;
typedef struct
{
   float ac_x;
   float ac_y;
   float ac_z;
   float gy_x;
   float gy_y;
   float gy_z;
}chw_angle;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
}siyuanshu;

// 二阶巴特沃斯低通滤波器 (Direct Form I biquad)
// 用于陀螺仪角速度数据的低通滤波，截止频率可配置
typedef struct
{
    float b0, b1, b2;       // 前馈系数
    float a1, a2;           // 反馈系数 (a0归一化为1)
    float x1, x2;           // 输入历史 (x[n-1], x[n-2])
    float y1, y2;           // 输出历史 (y[n-1], y[n-2])
} biquad_lpf;

extern my_qu sc;
extern quat_angle s_angle;
extern siyuanshu s_shu;
extern chw_angle cl_angle;
extern biquad_lpf gyro_lpf_x, gyro_lpf_y, gyro_lpf_z;  // 三轴陀螺仪低通滤波器
extern float imu660rb_acc_x_1;
extern float imu660rb_acc_y_1;
extern float imu660rb_acc_z_1;
extern float imu660rb_acc_x_0;
extern float imu660rb_acc_y_0;
extern float imu660rb_acc_z_0;
extern float imu660ra_gyro_x_1;
extern float imu660ra_gyro_y_1;
extern float imu660ra_gyro_z_1;
    

void quaternion_Init(my_qu*liu,quat_angle*aksq,siyuanshu*siyaun ,float kkp,float kki,float pit_time);
void my_quaternion(my_qu*liu,quat_angle*aksq,siyuanshu*siyaun);
void imu_calibration(void);
//void dtlb(void);
void ycl(void);

// 二阶巴特沃斯低通滤波器初始化和更新
void biquad_lpf_init(biquad_lpf *f, float cutoff_hz, float sample_hz);
float biquad_lpf_update(biquad_lpf *f, float input);
#endif