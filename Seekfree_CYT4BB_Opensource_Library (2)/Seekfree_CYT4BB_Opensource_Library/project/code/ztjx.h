/*********************************************************************************************************************
 * Name:pid.h
 *  
 * Created on:
 * Author:
********************************************************************************************************************/

#ifndef _ZTJX_H_
#define _ZTJX_H_
#define GET_SIZE  5

typedef struct
    {
     uint16 t_data[GET_SIZE];//存得到的数据
     uint8_t idx;//索引
     uint8_t cnt;//存了多少个数据
    }filter1;

//陀螺仪
extern float roll_x;//翻滚角
extern float pitch_y;//俯仰角
extern float yaw_z;//偏航角
extern float no ;
extern float PPXX_new;
extern float PPYY_new;
extern filter1 tof;

extern float imu_data_gyro_x ;
extern float imu_data_gyro_y ;
extern float imu_data_gyro_z;

extern float imu660rb_gyro_x_l;
extern float imu660rb_gyro_y_l;
extern float imu660rb_gyro_z_l;

extern float imu_data_gyro_x_1;
extern float imu_data_gyro_y_1;
extern float imu_data_gyro_z_1;

extern float imu660rc_gyro_x_2;
extern float imu660rc_gyro_y_2;
extern float imu660rc_gyro_z_2;

void EKF_Init(void);
void EKF_UpData(void);
void calibrate_gyro_rc(void);
void Rc_Get(void);

void filter_init(filter1 *f);
uint16 Get_Hight(filter1*f , uint16 new_data);
void imurb_get_values(void);
void imurc_get_values(void);
#endif 
