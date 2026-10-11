/*
 * find.h
 *
 *  Created on: 2025年4月26日
 *      Author: 39653
 */
#include "zf_common_headfile.h"
#ifndef CODE_FIND_H_
#define CODE_FIND_H
extern float screen_x;
extern float screen_y;
extern int xx;
extern int yy;
extern int center_x;  // 使用0.5f而不是除以2
extern int center_y;
extern uint16_t areamax;

extern short Connected_xmin[100];//最小横坐标
extern short Connected_ymin[100];//最小纵坐标
extern short Connected_xmax[100];//最大横坐标
extern short Connected_ymax[100];//最大纵坐标

extern uint8_t labelBuffer[MT9V03X_W*MT9V03X_H];
extern uint8_t labelcount;
void Seek_Beacon (void);
void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap);
extern uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
extern uint8 display_image[MT9V03X_H][MT9V03X_W];
void Set_image_twovalues(uint8 value);  //设置二值化
void Set_image_two(void);
void Bin_Image_Filter (void);//过滤噪点
//定义队列,队列-中两端都会变化,所以用头尾指针表示两端的变化
typedef struct Queue //队列属性
{
 struct QNode* first;
 struct QNode* last;
}Queue;//队列特别记下队首和队尾
void PushQueue(Queue *queue,int data);
int PopQueue(Queue *queue);
typedef struct QNode
{//队列节点属性
 int data;//当前的值
 struct QNode *next;//指向下一个的值
}QNode;
extern int16_t OFFSET0;    // 最远处，赛道中心值综合偏移量
extern int16_t OFFSET1;    // 第二格
extern int16_t OFFSET2;    // 最近，第三格
void eye_show (void);
float Square_detection (int16_t x,int16_t y);
#endif /* CODE_FIND_H_ */
