/*********************************************************************************************************************
 * Name:kaerman.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/

#ifndef _KAERMAN_H_
#define _KAERMAN_H_


typedef struct
{
  float X[3];
  float F[3][3];
  float P[3][3];
  float Q[3][3];
  float R[3][3];
  float Z[3];
  float I[3][3];
  float dt;// 采样时间
}kaerman_angle;

typedef struct
{
  float X[2];
  float F[2][2];
  float P[2][2];
  float Q[2][2];
  float R[2][2];
  //float Z;
  float I[2][2];
  float dt;// 采样时间
}kaerman_b;

extern kaerman_b guangliu_x;
extern kaerman_b guangliu_y;
extern kaerman_b gyro_x;
extern kaerman_b gyro_y;
extern kaerman_b gyro_z;
extern float xacc;
extern float yacc;
void kaerman_b_Init(kaerman_b*k,float qb_init,float qb_init_1,float rb_init,float rb_init_2,float pb_init,float dtb);
float kaermanb_Updata(kaerman_b*k,float pmw_data,float acc);
float kaermanb_Updata_1(kaerman_b*k,float v_data);

#endif