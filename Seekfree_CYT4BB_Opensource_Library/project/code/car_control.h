/*********************************************************************************************************************
 * Name:car_control.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "pid.h"
#include "ztjx.h"
#ifndef CODE_CAR_CONTROL_H_
#define CODE_CAR_CONTROL_H_

extern PIDController qianjin;
extern PIDController zuoyou;
extern PIDController xuanzhuan;
extern PIDController rv;
extern PIDController pv;
extern PIDController yv;
extern PIDController xv;
extern PIDController yyv;
extern PIDController back_x;
extern PIDController back_y;


extern int flag_get;
extern float car_qianjin_1 ;
extern float car_zuoyou_1 ;
extern float erro_xx;
extern float erro_yy;
extern float erro_xx_1;
extern float erro_yy_1;
extern float yy_count;
extern float xx_count;
extern float need_back_x;
extern float need_back_y;

extern float rcc_roll;
extern float rcc_pitch;
extern float rcc_yaw;


void car_pid(void);
void car_pid1 (void);
void car_Init(float kp1,float ki1,float kd1,float kp2,float ki2, float kd2,float kp3,float ki3,float kd3);
void motor_control(int8 lq,int8 rq,int8 lh,int8 rh);
void car_star(void);
void menghe(void);
uint8_t menghe_is_done(void);

#endif