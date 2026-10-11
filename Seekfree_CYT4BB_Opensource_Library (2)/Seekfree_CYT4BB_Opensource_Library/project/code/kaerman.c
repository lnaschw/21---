/*********************************************************************************************************************
 * Name:kaerman.h
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
kaerman_b guangliu_x;
kaerman_b guangliu_y;
float xacc;
float yacc;
kaerman_b gyro_x;
kaerman_b gyro_y;
kaerman_b gyro_z;

/*void kaerman_a_Init(kaerman_angle*k,float x_init,float q_init,float r_init,float p_init,float dt)
{
  for(int i =0;i<4;i++)//Xæÿ’Û
  {
     k->X[i] = x_init;
  }
  for(int i=0;i<4;i++)//Q‘Î…˘æÿ’Û
  {
    k->Q[i][i]=q_init;
  }
  for(int i=0;i<4;i++)//R‘Î…˘æÿ’Û
  {
    k->R[i][i]=r_init;
  }
  for(int i=0;i<4;i++)//Pæÿ’Û
  {
    k->P[i][i]=p_init;
  }
    k->dt = dt;
}*/

void kaerman_b_Init(kaerman_b*k,float qb_init,float qb_init_1,float rb_init,float rb_init_2,float pb_init,float dtb)//Ω·ππÃÂ£¨Q‘Î…˘æÿ’Û£¨R‘Î…˘æÿ’Û£¨Pæÿ’Û£¨÷–∂œ ±º‰
{
    k->X[0] = 0;
    k->X[1] = 0;
  //Q‘Î…˘æÿ’Û
    k->Q[0][0]=qb_init;
    k->Q[0][1]=0;
    k->Q[1][0]=0;
    k->Q[1][1]=qb_init;
  
  //R‘Î…˘æÿ’Û
  
    k->R[0][0]=rb_init;
    k->R[0][1]=0;
    k->R[1][0]=0;
    k->R[1][1]=rb_init_2;
  
  
    k->P[0][0]=pb_init;
    k->P[0][1]=0;
    k->P[1][0]=0;
    k->P[1][1]=pb_init;
  
    k->dt = dtb;
    k->F[0][0] = 1.0f;
    k->F[0][1] = k->dt;
    k->F[1][0] = 0.0f;
    k->F[1][1] = 1.0f;
    
    // ≥ı ºªØ I µ•Œªæÿ’Û (2x2)
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            if(i == j)
                k->I[i][j] = 1.0f;
            else
                k->I[i][j] = 0.0f;
        }
    }
    // ø…—°£∫≥ı ºªØ Z ≤‚¡øæÿ’Û
}

float kaermanb_Updata(kaerman_b*k,float pmw_data,float acc)
{
    
    float x_new[2];//X = FX
    x_new[0] = k->F[0][0]*k->X[0]+k->F[0][1]*k->X[1];
    x_new[1] = k->F[1][0]*k->X[0]+k->F[1][1]*k->X[1];//k->F[1][0]*k->X[0]+k->F[1][1]*k->X[1]
    
    //F*Pi*Ft+Q
    float pi[2][2];//F*Pi
    pi[0][0]=k->F[0][0]*k->P[0][0]+k->F[0][1]*k->P[1][0];
    pi[1][0]=k->F[1][0]*k->P[0][0]+k->F[1][1]*k->P[1][0];
    pi[0][1]=k->F[0][0]*k->P[0][1]+k->F[0][1]*k->P[1][1];
    pi[1][1]=k->F[1][0]*k->P[0][1]+k->F[1][1]*k->P[1][1];
    
    float Ft[2][2];
    Ft[0][0] = k->F[0][0];
    Ft[0][1] = k->F[1][0];
    Ft[1][0] = k->F[0][1];
    Ft[1][1] = k->F[1][1];
    
    //pi*Ft+Q
    float pi2[2][2];
    pi2[0][0]=pi[0][0]*Ft[0][0]+pi[0][1]*Ft[1][0]+k->Q[0][0];
    pi2[1][0]=pi[1][0]*Ft[0][0]+pi[1][1]*Ft[1][0]+k->Q[1][0];
    pi2[0][1]=pi[0][0]*Ft[0][1]+pi[0][1]*Ft[1][1]+k->Q[0][1];
    pi2[1][1]=pi[1][0]*Ft[0][1]+pi[1][1]*Ft[1][1]+k->Q[1][1];
    
    //Ki =Pi*Ht/(H*Pi*Ht+R)
    float K[2][2];
    float H[2][2]={{1,0},{0,1}};
    float hp[2][2];
    float hp0[2][2];
    float hp1[2][2];
    
    //Pi*Ht
    hp[0][0]=pi2[0][0]*H[0][0]+pi2[0][1]*H[1][0];
    hp[1][0]=pi2[1][0]*H[0][0]+pi2[1][1]*H[1][0];
    hp[0][1]=pi2[0][0]*H[0][1]+pi2[0][1]*H[1][1];
    hp[1][1]=pi2[1][0]*H[0][1]+pi2[1][1]*H[1][1];
    
    //H*Pi*Ht+R
    hp0[0][0]=H[0][0]*hp[0][0]+H[0][1]*hp[1][0]+k->R[0][0];
    hp0[1][0]=H[1][0]*hp[0][0]+H[1][1]*hp[1][0]+k->R[1][0];
    hp0[0][1]=H[0][0]*hp[0][1]+H[0][1]*hp[1][1]+k->R[0][1];
    hp0[1][1]=H[1][0]*hp[0][1]+H[1][1]*hp[1][1]+k->R[1][1];
    
    //  /(H*Pi*Ht+R)
    float det = hp0[0][0]*hp0[1][1] - hp0[0][1]*hp0[1][0];
    if(fabs(det) < 1e-6f) det = 1e-6f;
    hp1[0][0]=hp0[1][1]/det;//hp1[0][0]=hp0[1][1]/det
    hp1[0][1]=-hp0[0][1]/det;//hp1[0][1]=-hp0[0][1]/det
    hp1[1][0]=-hp0[1][0]/det;//hp1[1][0]=-hp0[1][0]/det
    hp1[1][1]=hp0[0][0]/det;//hp1[1][1]=hp0[0][0]/det
    //AIÃ´æ˝Àµµƒ∂‘
    
    //Ki =Pi*Ht/(H*Pi*Ht+R)
    K[0][0]=hp[0][0]*hp1[0][0]+hp[0][1]*hp1[1][0];
    K[1][0]=hp[1][0]*hp1[0][0]+hp[1][1]*hp1[1][0];
    K[0][1]=hp[0][0]*hp1[0][1]+hp[0][1]*hp1[1][1];
    K[1][1]=hp[1][0]*hp1[0][1]+hp[1][1]*hp1[1][1];
    
    //X=X+K(Z-HX)
    float HX[2];
    float z= pmw_data;
    float z1 = acc;
    HX[0]= H[0][0]*x_new[0]+H[0][1]*x_new[1];
    HX[1]= H[1][0]*x_new[0]+H[1][1]*x_new[1];
    float residual = z-HX[0];
    float residual0 = z1-HX[1];
    x_new[0] += K[0][0] * residual + K[0][1] * residual0*0.5;
    x_new[1] += K[1][0] * residual + K[1][1] * residual0*0.5;
    
    k->X[0]= x_new[0];
    k->X[1]= x_new[1];
    
    //P = (I-KiH)Pi,pi2
    float kh[2][2];
    kh[0][0]=K[0][0]*H[0][0]+K[0][1]*H[1][0];
    kh[0][1]=K[0][0]*H[0][1]+K[0][1]*H[1][1];
    kh[1][0]=K[1][0]*H[0][0]+K[1][1]*H[1][0];
    kh[1][1]=K[1][0]*H[0][1]+K[1][1]*H[1][1];
    kh[0][0]=k->I[0][0]-kh[0][0];
    kh[0][1]=k->I[0][1]-kh[0][1];
    kh[1][0]=k->I[1][0]-kh[1][0];
    kh[1][1]=k->I[1][1]-kh[1][1];
    k->P[0][0]=kh[0][0]*pi2[0][0]+kh[0][1]*pi2[1][0];
    k->P[0][1]=kh[0][0]*pi2[0][1]+kh[0][1]*pi2[1][1];
    k->P[1][0]=kh[1][0]*pi2[0][0]+kh[1][1]*pi2[1][0];
    k->P[1][1]=kh[1][0]*pi2[0][1]+kh[1][1]*pi2[1][1];

    return k->X[0];
}

float kaermanb_Updata_1(kaerman_b*k,float v_data)
{
    
    float x_new[2];//X = FX
    x_new[0] = k->F[0][0]*k->X[0]+k->F[0][1]*k->X[1];
    x_new[1] = k->F[1][0]*k->X[0]+k->F[1][1]*k->X[1];//k->F[1][0]*k->X[0]+k->F[1][1]*k->X[1]
    
    //F*Pi*Ft+Q
    float pi[2][2];//F*Pi
    pi[0][0]=k->F[0][0]*k->P[0][0]+k->F[0][1]*k->P[1][0];
    pi[1][0]=k->F[1][0]*k->P[0][0]+k->F[1][1]*k->P[1][0];
    pi[0][1]=k->F[0][0]*k->P[0][1]+k->F[0][1]*k->P[1][1];
    pi[1][1]=k->F[1][0]*k->P[0][1]+k->F[1][1]*k->P[1][1];
    
    float Ft[2][2];
    Ft[0][0] = k->F[0][0];
    Ft[0][1] = k->F[1][0];
    Ft[1][0] = k->F[0][1];
    Ft[1][1] = k->F[1][1];
    
    //pi*Ft+Q
    float pi2[2][2];
    pi2[0][0]=pi[0][0]*Ft[0][0]+pi[0][1]*Ft[1][0]+k->Q[0][0];
    pi2[1][0]=pi[1][0]*Ft[0][0]+pi[1][1]*Ft[1][0]+k->Q[1][0];
    pi2[0][1]=pi[0][0]*Ft[0][1]+pi[0][1]*Ft[1][1]+k->Q[0][1];
    pi2[1][1]=pi[1][0]*Ft[0][1]+pi[1][1]*Ft[1][1]+k->Q[1][1];
    
    //Ki =Pi*Ht/(H*Pi*Ht+R)
    float K[2][2];
    float H[2][2]={{1,0},{0,0}};
    float hp[2][2];
    float hp0[2][2];
    float hp1[2][2];
    
    //Pi*Ht
    hp[0][0]=pi2[0][0]*H[0][0]+pi2[0][1]*H[1][0];
    hp[1][0]=pi2[1][0]*H[0][0]+pi2[1][1]*H[1][0];
    hp[0][1]=pi2[0][0]*H[0][1]+pi2[0][1]*H[1][1];
    hp[1][1]=pi2[1][0]*H[0][1]+pi2[1][1]*H[1][1];
    
    //H*Pi*Ht+R
    hp0[0][0]=H[0][0]*hp[0][0]+H[0][1]*hp[1][0]+k->R[0][0];
    hp0[1][0]=H[1][0]*hp[0][0]+H[1][1]*hp[1][0]+k->R[1][0];
    hp0[0][1]=H[0][0]*hp[0][1]+H[0][1]*hp[1][1]+k->R[0][1];
    hp0[1][1]=H[1][0]*hp[0][1]+H[1][1]*hp[1][1]+k->R[1][1];
    
    //  /(H*Pi*Ht+R)
    float det = hp0[0][0]*hp0[1][1] - hp0[0][1]*hp0[1][0];
    if(fabs(det) < 1e-6f) det = 1e-6f;
    hp1[0][0]=hp0[1][1]/det;//hp1[0][0]=hp0[1][1]/det
    hp1[0][1]=-hp0[0][1]/det;//hp1[0][1]=-hp0[0][1]/det
    hp1[1][0]=-hp0[1][0]/det;//hp1[1][0]=-hp0[1][0]/det
    hp1[1][1]=hp0[0][0]/det;//hp1[1][1]=hp0[0][0]/det
    //AIÃ´æ˝Àµµƒ∂‘
    
    //Ki =Pi*Ht/(H*Pi*Ht+R)
    K[0][0]=hp[0][0]*hp1[0][0]+hp[0][1]*hp1[1][0];
    K[1][0]=hp[1][0]*hp1[0][0]+hp[1][1]*hp1[1][0];
    K[0][1]=hp[0][0]*hp1[0][1]+hp[0][1]*hp1[1][1];
    K[1][1]=hp[1][0]*hp1[0][1]+hp[1][1]*hp1[1][1];
    
    //X=X+K(Z-HX)
    float HX[2];
    float z= v_data;
    HX[0]= H[0][0]*x_new[0]+H[0][1]*x_new[1];
    HX[1]= H[1][0]*x_new[0]+H[1][1]*x_new[1];
    float residual = z-HX[0];
    x_new[0] += K[0][0] * residual ;
    x_new[1] += K[1][0] * residual ;
    
    k->X[0]= x_new[0];
    k->X[1]= x_new[1];
    
    //P = (I-KiH)Pi,pi2
    float kh[2][2];
    kh[0][0]=K[0][0]*H[0][0]+K[0][1]*H[1][0];
    kh[0][1]=K[0][0]*H[0][1]+K[0][1]*H[1][1];
    kh[1][0]=K[1][0]*H[0][0]+K[1][1]*H[1][0];
    kh[1][1]=K[1][0]*H[0][1]+K[1][1]*H[1][1];
    kh[0][0]=k->I[0][0]-kh[0][0];
    kh[0][1]=k->I[0][1]-kh[0][1];
    kh[1][0]=k->I[1][0]-kh[1][0];
    kh[1][1]=k->I[1][1]-kh[1][1];
    k->P[0][0]=kh[0][0]*pi2[0][0]+kh[0][1]*pi2[1][0];
    k->P[0][1]=kh[0][0]*pi2[0][1]+kh[0][1]*pi2[1][1];
    k->P[1][0]=kh[1][0]*pi2[0][0]+kh[1][1]*pi2[1][0];
    k->P[1][1]=kh[1][0]*pi2[0][1]+kh[1][1]*pi2[1][1];

    return k->X[0];
}

