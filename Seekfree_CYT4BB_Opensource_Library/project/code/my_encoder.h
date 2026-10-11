/*********************************************************************************************************************
 * Name:my_encoder.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "pid.h"
#ifndef _MY_ENCODER_H_
#define _MY_ENCODER_H_

extern int16 encoder_data_dir[4];
extern uint8 pit_state;
extern int16 encoder_count;

extern PIDController p1; 
extern PIDController p2;
extern PIDController p3;
extern PIDController p4;

void my_encoder_Init(float p1p,float p1i,float p1d,float p2p,float p2i,float p2d,float p3p,float p3i,float p3d,float p4p,float p4i,float p4d);
#endif