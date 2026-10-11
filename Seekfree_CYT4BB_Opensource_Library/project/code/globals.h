/*
 * globals.h
 *
 *  Created on: 2025年3月7日
 *      Author: Administrator
 */
//=====================================================用户代码的头文件=======================================================
#include "zf_device_imu660rb.h"
#include <stdint.h>
//=====================================================用户代码的头文件=======================================================
#ifndef CODE_GLOBALS_H_
#define CODE_GLOBALS_H_

extern float pwm_output;
extern float pwm_output_2;
extern float pwm_output_3;

extern float change_ax;
extern float change_ay;
extern float change_az;

extern float change_gx;
extern float change_gy;
extern float change_gz;

extern float g_yaw;
extern float g_pitch;
extern float g_roll;

extern float a_yaw;
extern float a_pitch;
extern float a_roll;

extern float final_yaw;
extern float final_pitch;
extern float final_roll;

extern float single_speed;
extern float target_turn;
extern float target_turn_2;

extern float set_speed ;

extern float eye_look_light;
//extern uint32 count;

/*extern PIDController speed_pid_2;              // 外环：速度环
extern PIDController angle_pid_2;              // 中环：角度环
extern PIDController angular_velocity_pid_2;   // 内环：角速度环
extern IncrementalPID angular_velocity_pid_3;

extern PIDController speed_pid;              //  外环：速度环
extern PIDController angle_pid;              //  中环：角度环
extern PIDController angular_velocity_pid;   //  内环：角速度环
extern IncrementalPID angular_velocity_pid_4;

extern PIDController turn_outer ;              //  外环
extern PIDController turn_inner ;              //  内环


extern float keep_going ;
extern float if_look_light ;
extern float if_turn ;

extern LowPassFilter gyro_x_filter, gyro_y_filter, gyro_z_filter;
extern LowPassFilter accel_x_filter, accel_y_filter, accel_z_filter;
*/

extern float eyelook;

extern float moment_yaw;
extern float target_turn_3;

extern float move_k1;
extern float move_k2;

extern float START_guosai;

extern float jixie_lingdian;
extern float KeyNum;


extern float camera_change;

extern float camera_delay;
extern float camera_yvzhi;

extern float qianhou_zhongzhi;
extern float change_zhongzhi;



#endif /* CODE_GLOBALS_H_ */
