/*********************************************************************************************************************
 * Name:ztjx.c
 *      姿态
 * Created on:
 * Author:
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "quaternion.h"

#define DEG_TO_RAD  ( 57.295779513082320876798154814105f)
#define GET_SIZE  5
#define GYRO_SCALE 14.3f

filter1 tof;

matrix_t exf_x;
matrix_t error;
EulerAngles euler_angle; 
imu_t imu_data = {0,0,0,0,0,0};
matrix_type r_yz = 0.006f;//0.001

const matrix_type q[4][4] = {{0.05, 0, 0, 0}, {0, 0.05, 0, 0}, {0, 0, 0.05, 0}, {0, 0, 0, 0.05 }};//0.95
const matrix_type r[3][3] = {{8000, 0, 0}, {0,8000 , 0}, {0, 0,8000}};//8000
const matrix_type p[4][4] = {{1000000, 0, 0, 0}, {0, 1000000, 0, 0}, {0, 0, 1000000, 0}, {0, 0, 0, 1000000}};//1000000
const matrix_type ekf[4] = {1, 0, 0, 0};

static matrix_t Q;
static matrix_t R;
static matrix_t P;
//static float yaw1;
//======
static float gyro_offset_roll = 0;
static float gyro_offset_pitch= 0;
static float gyro_offset_yaw= 0;
//float PPXX_new=0;
//float PPYY_new=0;
extern float PPXX;
extern float PPYY;
float PPXX_new;
float PPYY_new;
float no ;
//rb
float imu_data_gyro_x ;
float imu_data_gyro_y ;
float imu_data_gyro_z;

float imu660rb_gyro_x_l;
float imu660rb_gyro_y_l;
float imu660rb_gyro_z_l;
//rc
float imu_data_gyro_x_1;
float imu_data_gyro_y_1;
float imu_data_gyro_z_1;

float imu660rc_gyro_x_2;
float imu660rc_gyro_y_2;
float imu660rc_gyro_z_2;
// ICM660RC 专用 非阻塞延时采样校准（永不卡死）
void calibrate_gyro_rc(void)
{
 

}
//======
void EKF_Init(void)
{

	Matrix_From_Array(&exf_x, (const matrix_type*)ekf, 4, 1);
	Matrix_From_Array(&Q, (const matrix_type*)q, 4, 4);
	Matrix_From_Array(&R, (const matrix_type*)r, 3, 3);
	Matrix_From_Array(&P, (const matrix_type*)p, 4, 4);
}

static inline void quaternion_to_euler(void)
{   
    //float chw = 0.95;
    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    euler_angle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2)* DEG_TO_RAD;                                  // pitch
    euler_angle.roll = atan2(-2 * q1 * q1 - 2 * q2 * q2 + 1,2 * q2 * q3 + 2 * q0 * q1)* DEG_TO_RAD;   // roll
    euler_angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * DEG_TO_RAD;    // yaw
    
    
    /*euler_angle.pitch +=5;                                 // pitch
    euler_angle.roll +=84; // roll
    euler_angle.yaw +=18*/;
}

static float imu660rb_acc_x_l = 0;
static float imu660rb_acc_y_l = 0;
static float imu660rb_acc_z_l = 0;

/*void imurc_get_values(void)
{
  float K=0.02;  
  
    imu660rb_get_gyro();
    imu660rb_get_acc();
    
    float rb_acc_x = imu660rb_acc_transition(imu660rb_acc_x);
    float rb_acc_y = imu660rb_acc_transition(imu660rb_acc_y);
    float rb_acc_z = imu660rb_acc_transition(imu660rb_acc_z);
    
    imu660rb_gyro_x=(int)(imu660rb_gyro_x);
    imu660rb_gyro_y=(int)(imu660rb_gyro_y);
    imu660rb_gyro_z=(int)(imu660rb_gyro_z);
    
    
    //imu660rb_gyro_x=(int)(imu660rb_gyro_x);
    //imu660rb_gyro_y=(int)(imu660rb_gyro_y);
    //imu660rb_gyro_z=(int)(imu660rb_gyro_z);
    
    
    float rb_gyro_x = imu660rb_gyro_transition(imu660rb_gyro_x);  
    float rb_gyro_y = imu660rb_gyro_transition(imu660rb_gyro_y);
    float rb_gyro_z = imu660rb_gyro_transition(imu660rb_gyro_z);
    
    //一阶低通滤波，单位g/s
    imu_data.acc_x = K * rb_acc_x + (1 - K) * imu660rb_acc_x_l;
    imu_data.acc_y = K * rb_acc_y + (1 - K) * imu660rb_acc_y_l;
    imu_data.acc_z = K * rb_acc_z + (1 - K) * imu660rb_acc_z_l;
    
    imu660rb_acc_x_l = imu_data.acc_x;
    imu660rb_acc_y_l = imu_data.acc_y;
    imu660rb_acc_z_l = imu_data.acc_z;
    
    //陀螺仪角度转弧度
    imu_data.gyro_x = rb_gyro_x * PI / 180 ;
    imu_data.gyro_y = rb_gyro_y * PI / 180 ;
    imu_data.gyro_z = rb_gyro_z* PI / 180 ;
}*/

void imurb_get_values(void)
{
    float K=0.08;

    //一阶低通滤波，单位g/s
    imu_data_gyro_x = K * imu660rb_gyro_x/GYRO_SCALE + (1 - K) * imu660rb_gyro_x_l;
    imu_data_gyro_y = K * imu660rb_gyro_y/GYRO_SCALE + (1 - K) * imu660rb_gyro_y_l;
    imu_data_gyro_z = K * imu660rb_gyro_z/GYRO_SCALE + (1 - K) * imu660rb_gyro_z_l;
    
    imu660rb_gyro_x_l = imu_data_gyro_x;
    imu660rb_gyro_y_l = imu_data_gyro_y;
    imu660rb_gyro_z_l = imu_data_gyro_z;
    
    
    //printf("%f\n",imu660rb_gyro_z_l);
}

void imurc_get_values(void)
{
    float K=0.8;
      
    //一阶低通滤波，单位g/s
    imu_data_gyro_x_1 = K * imu660rc_gyro_x + (1 - K) * imu660rc_gyro_x_2;
    imu_data_gyro_y_1 = K * imu660rc_gyro_y + (1 - K) * imu660rc_gyro_y_2;
    imu_data_gyro_z_1 = K * imu660rc_gyro_z + (1 - K) * imu660rc_gyro_z_2;
    
    imu660rc_gyro_x_2 = imu_data_gyro_x_1;
    imu660rc_gyro_y_2 = imu_data_gyro_y_1;
    imu660rc_gyro_z_2 = imu_data_gyro_z_1;

}

void EKF_UpData(void)
{

  float dt=0.05;  
  
    float gx, gy, gz;
    //imu_get_values();
    gx = imu_data.gyro_x;
    gy = imu_data.gyro_y;
    gz = imu_data.gyro_z;

    matrix_t Z;

    Matrix_Init(&Z, 3, 1);

    Z.data[0][0] = (matrix_type)imu_data.acc_x;
    Z.data[1][0] = (matrix_type)imu_data.acc_y;
    Z.data[2][0] = (matrix_type)imu_data.acc_z;

    normalize_vector(&Z);

    matrix_type f[4][4]= {{1, -0.5f * gx * dt, -0.5f * gy * dt, -0.5f * gz * dt},
                          {0.5f * gx * dt, 1, 0.5f * gz * dt, -0.5f * gy * dt},
                          {0.5f * gy * dt, -0.5f * gz * dt, 1, 0.5f * gx * dt},
                          {0.5f * gz * dt, 0.5f * gy * dt, -0.5f * gx * dt, 1}};

    matrix_t F,FT;
    Matrix_From_Array(&F, (const matrix_type*)f, 4, 4);
    FT = Matrix_Transpose(&F);
    exf_x = multiply_matrices(&F, &exf_x);  //X = F * X;
    //matrix_t temp_result = multiply_matrices(&F, &exf_x);
    //Matrix_Copy(&exf_x, &temp_result);
    normalize_vector(&exf_x);
    
    
    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    matrix_type h[3][4]={{-2 * q2, 2 * q3, -2 * q0, 2 * q1},
                          {2 * q1, 2 * q0, 2 * q3, 2 * q2},
			  {2 * q0, -2 * q1, -2 * q2, 2 * q3}};

    matrix_t H, HT;
    Matrix_From_Array(&H, (const matrix_type*)h, 3, 4);
    HT = Matrix_Transpose(&H);
    matrix_t PK_;

    //PK_ = F * P(K - 1) * FT + Q;
    PK_ = multiply_matrices(&F, &P);       //F * P;
    PK_ = multiply_matrices(&PK_, &FT);    //F * P * FT;
    //matrix_t temp_resul = multiply_matrices(&PK_, &FT);
    //Matrix_Copy(&PK_, &temp_resul);
    P = add_matrices(&PK_, &Q);            //F * P * FT + Q;


    //DK_ = H * PK_ * HT + R;
    matrix_t DK, invDK;
    DK = multiply_matrices(&H, &P);
    DK = multiply_matrices(&DK, &HT);
    //matrix_t temp_result1 = multiply_matrices(&DK, &HT);
    //Matrix_Copy(&DK, &temp_result1);
    DK = add_matrices(&DK, &R); 

    if(inverse_matrix(&DK, &invDK))
    {
    	quaternion_to_euler();
    	return;
    }

    //ek = Z - H * X;
    matrix_t EK, EKT;
    EK = multiply_matrices(&H, &exf_x);     //H * X;
    EK = subtract_matrices(&Z, &EK);        //Z - HX;
    EKT = Matrix_Transpose(&EK);

    //r = EKT * invDK * EK;
    error = multiply_matrices(&EKT, &invDK);
    error = multiply_matrices(&error, &EK);
    //matrix_t temp_result3 = multiply_matrices(&error, &EK);
    //Matrix_Copy(&error, &temp_result3);

    if(error.data[0][0] > r_yz)
    {
    	quaternion_to_euler();
    	return;
    }

    //Kk = M * P * HT * invDK;
    matrix_t Kk;
    Kk = multiply_matrices(&P, &HT);
    Kk = multiply_matrices(&Kk, &invDK);
    //matrix_t temp_result4 = multiply_matrices(&Kk, &invDK);
    //Matrix_Copy(&Kk, &temp_result4);

    //X = X_ + Kk * Ek; 
    matrix_t temp;
    temp = multiply_matrices(&Kk, &EK);
    exf_x = add_matrices(&exf_x, &temp);
    normalize_vector(&exf_x);

    //P = (I - Kk * H) * PK_;
    matrix_t I;
    Matrix_Identity(&I, 4);
    temp = multiply_matrices(&Kk, &H);
    temp = subtract_matrices(&I, &temp);
    P = multiply_matrices(&temp, &P);
     //matrix_t temp_result6 = multiply_matrices(&temp, &P);
    //Matrix_Copy(&P, &temp_result6);
        quaternion_to_euler();
        
    printf("%f,%f,%f\n",euler_angle.yaw - no,euler_angle.pitch,euler_angle.roll);
}                                            //+24-6+1                                   //+83

//F矩阵是通过陀螺仪的预测值
//H矩阵是通过四元数的理论观测值

//PK_ = F * P(K - 1) * FT + Q;  / PK_是协方差矩阵，/ Q是过程噪声矩阵
//DK_ = H * PK_ * HT + R;       / H：观测矩阵（雅可比） / PK_：预测状态协方差矩阵 / HT：H的转置 / R：观测噪声协方差矩阵（加速度计噪声） / DK_：创新协方差矩阵（3×3）
//ek = Z - H * X;               / Z：实际测量向量（3×1，加速度计测量） / H：观测矩阵（3×4） / X：预测状态向量（4×1，四元数） / ek：残差向量（3×1）
//Kk = M * P * HT * invDK;      /P：预测状态协方差矩阵（4×4） / H：观测矩阵（3×4） / HT：H的转置（4×3） / invDK：创新协方差DK的逆（3×3） /  Kk：卡尔曼增益矩阵（4×3）
//X = X_ + Kk * Ek;             /X_：预测状态（预测的四元数） /  Kk：卡尔曼增益矩阵（4×3） /  Ek：残差向量（3×1） /  X：更新后的状态（最终的四元数）
//P = (I - Kk * H) * PK_;       /更新

void Rc_Get(void)
{
  euler_angle.roll = imu660rc_roll;//gyro_offset_roll
  euler_angle.pitch = imu660rc_pitch;
  euler_angle.yaw = imu660rc_yaw;
  printf("%f,%f,%f\n",euler_angle.roll ,euler_angle.pitch,s_angle.agz);
  //printf("%f,%f,%f\n",euler_angle.roll ,euler_angle.pitch,euler_angle.yaw);
  //printf("%f\n",euler_angle.yaw);
}
void filter_init(filter1 *f)
{
    for(int i = 0; i<GET_SIZE; i++)
      f->t_data[i]=0;
      f->idx = 0;
      f->cnt = 0;
}

uint16 Get_Hight(filter1*f , uint16 new_data)
{
    f->t_data[f->idx] = new_data;
    f->idx = (f->idx +1)%GET_SIZE;
    if(f->cnt < GET_SIZE)
    {
      f->cnt++;
    }
    //复制一遍
    uint16 temp[GET_SIZE];
    uint16 temp1;
    for(int i = 0; i<f->cnt;i++)
    {
      temp[i] = f->t_data[i];
    }
    //冒泡排序
    for(int i = 0;i<f->cnt;i++)
    {
      for(int j = i; j<f->cnt-1;j++)
      {
        if(temp[j+1]>temp[i])
        {
          temp1 = temp[i];
          temp[i] = temp[j+1];
          temp[j+1] = temp1;
        }
      }
    }
    //计算
    uint16 sum=0;
    uint16 num;
    if(f->cnt<GET_SIZE)
    {
      num = f->cnt;
      for(int i=0;i<num;i++)
      {
        sum+=temp[i];
      }
    }
    else
    {
      sum = temp[1]+temp[2]+temp[3];
      num = 3;
    }
    return sum / num;
}

