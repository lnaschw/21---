/*
 * image.c
 *
 *  Created on: 2025年3月5日
 *      Author: dell
 */
#include "zf_common_headfile.h"
#include "image.h"
#include "find.h"

uint8 base_image[MT9V03X_H][MT9V03X_W];

/************************************************************************
*  函数名称：void Get_mt9v03x_image_BandW (unsigned char mode)
*  功能说明：图像二值化到mt9v03x_image_BandW[][]
*  参数说明：mode  ：
*    0：使用大津法阈值
*    1：使用平均阈值
*    2: sobel 算子改进型  手动阈值，同时输出改为提取边沿的图像
*    3：sobel 算子改进型   动态阈值，同时输出改为提取边沿的图像
*  函数返回：无
*  修改时间：2022年3月29日
*  备    注：  Get_mt9v03x_image_BandW(0); //使用大津法二值化
************************************************************************/
void Get_mt9v03x_image_BandW (unsigned char mode)
{
  unsigned short i = 0, j = 0;
  unsigned short Threshold = 0;
  unsigned long tv = 0;

  if (mode == 0)
  {
      Threshold = GetOSTU(mt9v03x_image);  //大津法阈值
  }
  if (mode == 1)
  {
    //累加
    for (i = 0; i < MT9V03X_H; i++)
    {
      for (j = 0; j < MT9V03X_W; j++)
      {
        tv += mt9v03x_image[i][j];   //累加
      }
    }
    Threshold =(unsigned short)(tv / MT9V03X_H / MT9V03X_W);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
    Threshold = Threshold+10;      //此处阈值设置，根据环境的光线来设定
  }
  else if (mode == 2)
  {
    Threshold = 80;                          //手动调节阈值
    lq_sobel(mt9v03x_image, mt9v03x_image_BandW, (unsigned char) Threshold);

    return;

  }
  else if (mode == 3)
  {
    lq_sobelAutoThreshold(mt9v03x_image, mt9v03x_image_BandW);  //动态调节阈值
    return;
  }

  /* 二值化 */
   for (i = 0; i < MT9V03X_H; i++)
  {
    for (j = 0; j < MT9V03X_W; j++)
    {
      if (mt9v03x_image[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
          mt9v03x_image_BandW[i][j] = 255;
      else
          mt9v03x_image_BandW[i][j] = 0;
    }
  }
}


/************************************************************************
*  函数名称：short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W])
*  功能说明：大津法求阈值大小
*  参数说明：tmImage ： 图像数据
*  函数返回：无
*  修改时间：2022年3月29日
*  备    注：  GetOSTU(mt9v03x_image);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
************************************************************************/
short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W])
{
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  signed short MinValue, MaxValue;
  signed short Threshold = 0;
  unsigned char HistoGram[256];
  unsigned long colorcount=0;//灰度个数

  for (j = 0; j < 256; j++)
    HistoGram[j] = 0; //初始化灰度直方图

  for (j = 0; j < MT9V03X_H; j++)
  {
    for (i = 0; i < MT9V03X_W; i++)
    {
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }
  for(i=0;i<256;i++)
  {
    if(HistoGram[i]>0)
    {
      colorcount++;
    }
  }
      
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--); //获取最大灰度的值

  if (MaxValue == MinValue)
    return MaxValue;         // 图像中只有一个颜色
  if (MaxValue-MinValue <=5 &&colorcount==2)
    return (MinValue+MaxValue)/2;        // 图像中只有二个颜色

  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  像素总数

  Pixelshortegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //前景像素点数
    PixelFore = Amount - PixelBack;           //背景像素点数
    OmegaBack = (float) PixelBack / Amount;  //前景像素百分比
    OmegaFore = (float) PixelFore / Amount;  //背景像素百分比
    PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
    PixelshortegralFore = Pixelshortegral - PixelshortegralBack;   //背景灰度值
    MicroBack = (float) PixelshortegralBack / PixelBack;          //前景灰度百分比
    MicroFore = (float) PixelshortegralFore / PixelFore;          //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差===前景像素个数*背景像素个数*（前景灰度比-背景灰度值比）的平方。
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
}


/*************************************************************************
*  函数名称：lq_sobel
*  功能说明：基于soble边沿检测算子的一种边沿检测
* @param    imageIn    输入数组
*           imageOut   输出数组      保存的二值化后的边沿信息
*           Threshold  阈值
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：无
*************************************************************************/
void lq_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
{
  //* 卷积核大小 // 
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]                   // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];        // {-1, 0, 1};

      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //-1, -1, -1,
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]                   // 0,  0,  0,
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];       //  1,  1,  1;

      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]             //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1]; //  1,  1,  0

      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]              // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];  //  0,  1,  1

      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
      temp[2] = abs(temp[2]);
      temp[3] = abs(temp[3]);

      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }

      if (temp[0] > Threshold)
      {
        imageOut[i][j] = 255;//1
      }
      else
      {
        imageOut[i][j] = 0;
      }
    }
  }
}

/*************************************************************************
*  函数名称：lq_sobel
*  功能说明：基于soble边沿检测算子的一种自动阈值边沿检测
* @param    imageIn    输入数组
*           imageOut   输出数组      保存的二值化后的边沿信息 *
*  函数返回：void
*  修改时间：2020年3月10日
*  备    注：注意需要使用  带与非门版（白色）转接座，或者新版摄像头模块
*************************************************************************/
void lq_sobelAutoThreshold (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W])
{
  //* 卷积核大小 //
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd   = MT9V03X_W - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd   = MT9V03X_H - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
      temp[2] = abs(temp[2]);
      temp[3] = abs(temp[3]);

      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }

      // 使用像素点邻域内像素点之和的一定比例    作为阈值  //
      temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
        + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
          + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

      if (temp[0] > temp[3] / 12.0f)
      {
        imageOut[i][j] = 1;
      }
      else
      {
        imageOut[i][j] = 0;
      }

    }
  }
}

void erosion_operation(unsigned char imageIn[MT9V03X_H][MT9V03X_W])
{
   int i;
   int j;
   for(i=1;i<MT9V03X_H-1;i++){
      for(j=1;j<MT9V03X_W-1;j++)
      {
        if(imageIn[i-1][j]+imageIn[i][j+1]+imageIn[i+1][j]+imageIn[i][j-1]==0)
        {
          imageIn[i][j]=0;
        }
       
      }
     
   }
}