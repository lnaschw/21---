/*
 * image.h
 *
 *  Created on: 2025年3月5日
 *      Author: dell
 */
#include "zf_common_headfile.h"
#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

extern uint8 base_image[MT9V03X_H][MT9V03X_W];//原始图像（用mecpy)
extern uint8 img_threshold;//输出阈值
short GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W]);
void lq_sobel(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold);
void lq_sobelAutoThreshold(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W]);
extern unsigned short Threshold;
extern volatile uint8_t dotcnt;
void quanzhong_light(void);
int find_max(void);
void quanzhong_light_panduan(void);
void Get_mt9v03x_image_BandW (unsigned char mode);
void erosion_operation(unsigned char imageIn[MT9V03X_H][MT9V03X_W]);
#endif /* CODE_IMAGE_H_ */
