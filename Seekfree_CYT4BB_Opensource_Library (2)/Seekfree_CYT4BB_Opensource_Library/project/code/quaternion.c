/*********************************************************************************************************************
 * Name:quaternion.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "kaerman.h"
#include "quaternion.h"
my_qu sc;
quat_angle s_angle;
siyuanshu s_shu;
chw_angle cl_angle;
biquad_lpf gyro_lpf_x, gyro_lpf_y, gyro_lpf_z;  // 三轴陀螺仪低通滤波器实例
float imu660rb_acc_x_1 =0;
float imu660rb_acc_y_1 =0;
float imu660rb_acc_z_1 =0;
float imu660rb_acc_x_0 = 0;
float imu660rb_acc_y_0 = 0;
float imu660rb_acc_z_0 = 0;
float imu660ra_gyro_x_1=0;
float imu660ra_gyro_y_1=0;
float imu660ra_gyro_z_1=0;

float gyro_offset_x = 0.0f;                                     // 存放X轴陀螺仪开机静止时的本底噪声偏差值
float gyro_offset_y = 0.0f;                                     // 存放Y轴陀螺仪开机静止时的本底噪声偏差值
float gyro_offset_z = 0.0f; 

#define GYRO_SCALE 14.3f
//16.4f
#define ACC_SCALE  4098.0f
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     二阶巴特沃斯低通滤波器初始化 (Direct Form I)
// 参数说明     f           滤波器结构体指针
// 参数说明     cutoff_hz   截止频率 (Hz)，建议陀螺仪30~50Hz
// 参数说明     sample_hz   采样频率 (Hz)，对应PIT中断周期
// 使用示例     biquad_lpf_init(&gyro_lpf_x, 40.0f, 500.0f);
//-------------------------------------------------------------------------------------------------------------------
void biquad_lpf_init(biquad_lpf *f, float cutoff_hz, float sample_hz)
{
    float w0 = 2.0f * 3.14159265359f * cutoff_hz / sample_hz;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * 0.70710678118f);  // Q = 1/sqrt(2), Butterworth

    float a0 = 1.0f + alpha;
    f->b0 = ((1.0f - cos_w0) / 2.0f) / a0;
    f->b1 = (1.0f - cos_w0) / a0;
    f->b2 = ((1.0f - cos_w0) / 2.0f) / a0;
    f->a1 = (-2.0f * cos_w0) / a0;
    f->a2 = (1.0f - alpha) / a0;

    // 清零历史状态
    f->x1 = 0.0f; f->x2 = 0.0f;
    f->y1 = 0.0f; f->y2 = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     二阶巴特沃斯低通滤波器更新 (Direct Form I)
// 参数说明     f           滤波器结构体指针
// 参数说明     input       当前输入采样值
// 返回参数     float       滤波后的输出值
// 差分方程     y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
//-------------------------------------------------------------------------------------------------------------------
float biquad_lpf_update(biquad_lpf *f, float input)
{
    float output = f->b0 * input
                 + f->b1 * f->x1
                 + f->b2 * f->x2
                 - f->a1 * f->y1
                 - f->a2 * f->y2;

    // 更新历史状态
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;

    return output;
}

void quaternion_Init(my_qu*liu,quat_angle*aksq,siyuanshu*siyaun ,float kkp,float kki,float pit_time)
{
  liu->kp = kkp;
  liu->ki = kki;
  liu->qdt = pit_time;
  aksq->agx=0;
  aksq->agy=0;
  aksq->agz=0;
  siyaun->q0 = 1;
  siyaun->q1 = 0;
  siyaun->q2 = 0;
  siyaun->q3 = 0;

  // 初始化三轴陀螺仪巴特沃斯低通滤波器
  // 采样率 = 1/pit_time, 截止频率40Hz (陀螺仪常用值, 平衡噪声抑制与动态响应)
  float sample_rate = 1.0f / pit_time;  // 例如: 1/0.002 = 500Hz
  biquad_lpf_init(&gyro_lpf_x, 40.0f, sample_rate);
  biquad_lpf_init(&gyro_lpf_y, 40.0f, sample_rate);
  biquad_lpf_init(&gyro_lpf_z, 40.0f, sample_rate);
}

void my_quaternion(my_qu*liu, quat_angle*aksq, siyuanshu*siyaun)
{
    // 1. 修复低通滤波器状态丢失的问题 (使用 static)
    float my_kp = liu->kp;
    float my_magic = 1.0f;
  
    static float last_acc_x = 0, last_acc_y = 0, last_acc_z = 0;
    static float exInt = 0,eyInt = 0,ezInt = 0;
    
    float ff_gx = -(imu660rb_gyro_x-gyro_offset_x);
    float ff_gy = (imu660rb_gyro_y-gyro_offset_y);
    float ff_gz = -(imu660rb_gyro_z-gyro_offset_z);
    
    //printf("%f,%f,%f\n",ff_gx,ff_gy,ff_gz);
    // 静止时在线追踪 gyro_offset_z，吃掉温漂
    // 触发条件：三轴 raw 残留都 < 20 raw（≈1.4°/s），认为车真静止
    // 追踪步长 0.001：每秒拉拢 ~40%，足够跟上温漂又不会被瞬时噪声带偏
    // 同时设 is_still 标志，下面 dq3 计算后用它做 ZUPT 兜底（静止时强制 dq3=0）
    
    
    /*static uint8 is_still = 0;
    if(fabsf(ff_gx) < 20.0f && fabsf(ff_gy) < 20.0f && fabsf(ff_gz) < 20.0f)
    {
        gyro_offset_z += ((float)imu660rb_gyro_z - gyro_offset_z) * 0.001f;
        is_still = 1;
    }
    else
    {
        is_still = 0;
    }*/

    if(fabsf(ff_gx) < 3.0f) ff_gx = 0.0f;
    if(fabsf(ff_gy) < 3.0f) ff_gy = 0.0f;
    // 静止零飘修复：原 3 raw 偏小，扩大到 12 raw (≈0.84°/s) 兜底典型温漂残留
    //              即使在线追踪没收敛时也能切掉小残留；代价是 <0.84°/s 慢转 yaw 不响应
    if(fabsf(ff_gz) < 12.0f) ff_gz = 0.0f;
    
    float kk = 0.05f;
    
    float raw_ax = -(imu660rb_acc_x/ACC_SCALE);
    float raw_ay = (imu660rb_acc_y/ACC_SCALE);
    float raw_az = -(imu660rb_acc_z/ACC_SCALE);
    
    //printf("%f\n",raw_ay);
    
    // 加速度低通滤波
    float ax = (kk * last_acc_x + (1.0f - kk) * raw_ax);
    float ay =  kk * last_acc_y + (1.0f - kk) * raw_ay; // 确认正负号是否符合你的坐标系
    float az = (kk * last_acc_z + (1.0f - kk) * raw_az);
    last_acc_x = ax; last_acc_y = ay; last_acc_z = az;
    //printf("%f,%f,%f\n",raw_ax,raw_ay,raw_az);
    
    // 陀螺仪角速度二阶巴特沃斯低通滤波 (替代原来弱一阶滤波 kk1=0.1)
    // 截止频率 40Hz @ 500Hz采样率, -12dB/oct衰减, 有效抑制高频噪声
    float gx_deg = biquad_lpf_update(&gyro_lpf_x, ff_gx);
    float gy_deg = biquad_lpf_update(&gyro_lpf_y, ff_gy);
    float gz_deg = biquad_lpf_update(&gyro_lpf_z, ff_gz);

    // 转换为弧度制
    float gx = (gx_deg/GYRO_SCALE) * 0.01745329f;
    float gy = (gy_deg/GYRO_SCALE) * 0.01745329f;
    float gz = (gz_deg/GYRO_SCALE) * 0.01745329f;

    float ckx, cky, ckz;
    float ex, ey;//ez
    float norm;

    float q0 = siyaun->q0;
    float q1 = siyaun->q1;
    float q2 = siyaun->q2;
    float q3 = siyaun->q3;

       
        norm = sqrtf(ax*ax+ay*ay+az*az);
        float norm_err = fabsf(norm - 1.0f);
        //printf("%f\n",norm_err);

        // 加速度计信任度: norm越接近1.0越可信, 偏离越多越不可信
        // trust = 1.0 (静止/匀速) → 0.0 (大幅加减速/离心)
        float acc_trust = 1.0f;
        if(norm_err < 0.05f)
        {
            acc_trust = 1.0f;                        // 几乎纯重力, 完全信任
        }
        else if(norm_err < 0.15f)
        {
            acc_trust = 1.0f - (norm_err - 0.05f) / 0.10f;  // 线性衰减 1.0→0.0
        }
        else
        {
            acc_trust = 0.0f;                        // 外部加速度太大, 不信任
        }

        // 陀螺仪角速率越大, 运动越剧烈, 进一步降低加速度计信任度
        float gyro_rate = sqrtf(gx*gx + gy*gy + gz*gz);
        float gyro_gx = fabs(gx);
        float gyro_gy = fabs(gy);
        if(gyro_rate > 0.3f)
        {
            // 角速率 > 0.3rad/s(≈17°/s)时, 运动信任度线性衰减
            float gyro_trust = 1.0f;
            if(gyro_rate < 1.5f)
                gyro_trust = 1.0f - (gyro_rate - 0.3f) / 1.2f;
            else
                gyro_trust = 0.0f;

            // 取两种信任度的较小值
            if(gyro_trust < acc_trust) acc_trust = gyro_trust;
        }

        // 对Yaw轴更保守: X/Y轴有大角速度时进一步降权
        if(gyro_gx > 0.4f || gyro_gy > 0.4f)
        {
            acc_trust *= 0.3f;
            my_magic = 0.1f;  // 大幅摆动时降低yaw四元数更新权重
        }

     if (acc_trust > 0.01f)  // 只要有微弱信任就做修正 (原来完全切断太粗暴)
    {
        ax /= norm;
        ay /= norm;
        az /= norm;

        // 提取重力估计向量
        ckx = 2.0f * (q1*q3 - q0*q2);
        cky = 2.0f * (q0*q1 + q2*q3);
        ckz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 叉积求误差
        ex = (ay*ckz - az*cky);
        ey = (az*ckx - ax*ckz);

        // 误差限幅: 防止单次野值污染姿态 (0.3≈17°误差)
        if(ex > 0.3f) ex = 0.3f; else if(ex < -0.3f) ex = -0.3f;
        if(ey > 0.3f) ey = 0.3f; else if(ey < -0.3f) ey = -0.3f;

        // 用信任度缩放比例项 (信任低时减小修正力度)
        my_kp = liu->kp * acc_trust;
        if(my_kp < 0.001f) my_kp = 0.001f;  // 保底极小修正

        // 积分项只在高度信任时累加 (norm_err < 0.05, trust ≈ 1.0)
        // 防止边界状态下的错误积分累积
        if(acc_trust > 0.9f)
        {
            exInt += ex * liu->ki * liu->qdt;
            eyInt += ey * liu->ki * liu->qdt;
        }
        ezInt = 0;

        // 积分限幅
        if(exInt > 3.0f) exInt = 3.0f; else if(exInt < -3.0f) exInt = -3.0f;
        if(eyInt > 3.0f) eyInt = 3.0f; else if(eyInt < -3.0f) eyInt = -3.0f;
        if(ezInt > 3.0f) ezInt = 3.0f; else if(ezInt < -3.0f) ezInt = -3.0f;
    }
      else
      {
        // 完全不可信: 纯陀螺仪积分, 保留原有积分不做清零
        ex = 0.0f;
        ey = 0.0f;
        my_kp = 0.001f;
        // 大幅摆动时降低yaw四元数更新权重, 防止coning误差累积
        if(gyro_gx > 0.4f || gyro_gy > 0.4f) my_magic = 0.1f;
      }

        // PI 补偿角速度 (使用信任度自适应调整后的 my_kp)
        gx = gx + (my_kp * ex) + exInt;
        gy = gy + (my_kp * ey) + eyInt;
        //gz = gz + (liu->kp * ez*0.05)+ezInt; // 纯靠陀螺仪积分
        gz = gz;


    // 四元数更新 (龙格库塔法)
    float halfT = liu->qdt * 0.5f;
    float dq0 = (-q1*gx - q2*gy - q3*gz) * halfT;
    float dq1 = ( q0*gx + q2*gz - q3*gy) * halfT;
    float dq2 = ( q0*gy - q1*gz + q3*gx) * halfT;
    float dq3 = (( q0*gz + q1*gy - q2*gx) * halfT)*my_magic;
    // 关键修复：晃动期间冻结 dq3，根治"晃完跳N度且一次比一次大"
    // 机制：dq3 的 q1*gy - q2*gx 项在多轴循环运动下一阶 Euler 积分留 O(dt) 残差
    // (coning error)，再叠加 MEMS 陀螺 g-sensitivity 让 gz 在振动下读出虚假值，
    // 每个 shake 周期 q3 净增一点点 → N 次累加。冻结 dq3 后 q3 不再积假账。
    // 代价：剧烈晃动期间主动 yaw 指令不响应；阈值用现成的 0.5 复用 shake 检测。
    /*if(gyro_rate > 0.8f) {
        dq3 = 0.0f;
    }*/
    // ZUPT：静止时冻结 dq0/dq1/dq2，dq3 保留正常计算（用户要求 dq3 不冻结）
    /*if(is_still)
    {
        dq0 = 0.0f;
        dq1 = 0.0f;
        dq2 = 0.0f;
    }*/

    siyaun->q0 += dq0;
    siyaun->q1 += dq1;
    siyaun->q2 += dq2;
    siyaun->q3 += dq3;

    // 四元数归一化
    norm = sqrtf(siyaun->q0*siyaun->q0 + siyaun->q1*siyaun->q1 + siyaun->q2*siyaun->q2 + siyaun->q3*siyaun->q3);
    siyaun->q0 /= norm;
    siyaun->q1 /= norm;
    siyaun->q2 /= norm;
    siyaun->q3 /= norm;

    // 计算欧拉角
    q0 = siyaun->q0; q1 = siyaun->q1; q2 = siyaun->q2; q3 = siyaun->q3;
    aksq->agx = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.2957795f;
    aksq->agy = asinf(2.0f*(q0*q2 - q1*q3)) * 57.2957795f;
    aksq->agz = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.2957795f;
}


void imu_calibration(void)                                      
{                                                               
    // 防止单片机启动太快，读到全是无效的0导致校准失效                                       

    int32 sum_gx = 0, sum_gy = 0, sum_gz = 0;                   // 定义累加变量防溢出
    for(int i = 0; i <1000; i++)                                // 开启连续500次采样
    {                                                           
        imu660rb_get_gyro();                                    // 抓取底层陀螺仪数据
        sum_gx += imu660rb_gyro_x;                                  
        sum_gy += imu660rb_gyro_y;                                  
        sum_gz += imu660rb_gyro_z;                                  
        system_delay_ms(1);                                     // 每次延时2毫秒，总共耗时1秒钟
    }                                                           
    
    gyro_offset_x = (float)sum_gx / 1000.0f;                     // 计算真正的静止绝对零偏
    gyro_offset_y = (float)sum_gy / 1000.0f;                     
    gyro_offset_z = (float)sum_gz / 1000.0f;                     
}                                             

void ycl(void)
{
static float acc_alpha = 0.2f;    
cl_angle.ac_x=acc_alpha*cl_angle.ac_x+(1-acc_alpha)*imu660rb_acc_x;
cl_angle.ac_y=acc_alpha*cl_angle.ac_y+(1-acc_alpha)*imu660rb_acc_y;
cl_angle.ac_z=acc_alpha*cl_angle.ac_z+(1-acc_alpha)*imu660rb_acc_z;

//cl_angle.gy_x = imu660rb_gyro_x;
//cl_angle.gy_y = imu660rb_gyro_y;
cl_angle.gy_z = imu660rb_gyro_z;

static float gyro_filtered_z = 0; //gyro_filtered_x = 0, gyro_filtered_y = 0, 
float raw_gyro_z_dps = (imu660rb_gyro_z - gyro_offset_z) / 14.30f;
float FILTER_ALPHA = 0.2f;
if(raw_gyro_z_dps > -4.0f && raw_gyro_z_dps < 4.0f)
    {
        // 频率降回 5ms 后，把追踪阻尼恢复到 0.0005f，稳稳吃掉温漂！
        gyro_offset_z += (imu660rb_acc_z - gyro_offset_z) * 0.0005f;
    }

    // 【强制静区】：角速度小于 4.0 度/秒，直接一刀切判定为 0！
    if(raw_gyro_z_dps > -4.0f && raw_gyro_z_dps < 4.0f)
    {
        cl_angle.ac_z = 0.0f;                                              
        gyro_filtered_z = 0.0f;                                             
    }
    else
    {
        float gyro_z_rad = raw_gyro_z_dps * 0.01745329f;
        gyro_filtered_z = FILTER_ALPHA * gyro_filtered_z + (1.0f - FILTER_ALPHA) * gyro_z_rad;
        cl_angle.gy_z = gyro_filtered_z;
    }
    //gyro_filtered_x = FILTER_ALPHA * gyro_filtered_x + (1.0f - FILTER_ALPHA) * cl_angle.gy_x;   
    //gyro_filtered_y = FILTER_ALPHA * gyro_filtered_y + (1.0f - FILTER_ALPHA) * cl_angle.gy_y;  
    //cl_angle.gy_x=gyro_filtered_x;
    //cl_angle.gy_y=gyro_filtered_y;
}
