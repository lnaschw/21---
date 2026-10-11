/*********************************************************************************************************************
 * Name:my_encoder.c
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

int16 encoder_data_dir[4] = {0};
uint8 pit_state = 0;
int16 encoder_count =0;

PIDController p1;
PIDController p2;
PIDController p3;
PIDController p4;

void my_encoder_Init(float p1p,float p1i,float p1d,float p2p,float p2i,float p2d,float p3p,float p3i,float p3d,float p4p,float p4i,float p4d)
{
    
    PID_Init(&p1,p1p,p1i,p1d);
    PID_Init(&p2,p2p,p2i,p2d);
    PID_Init(&p3,p3p,p3i,p3d);
    PID_Init(&p4,p4p,p4i,p4d);
    
    encoder_count = (encoder_data_dir[0]+encoder_data_dir[1]+encoder_data_dir[2]+encoder_data_dir[3])/4;
}
