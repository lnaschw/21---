/*
 * find.c
 *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
 *  Created on: 2025年4月26日
 *      Author: 39653
 */
#include "find.h"
#include "image.h"
#include "string.h"
#include "zf_device_mt9v03x.h"
#include "globals.h"
uint8 mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W];
uint8 display_image[MT9V03X_H][MT9V03X_W];
uint8_t labelBuffer[MT9V03X_W*MT9V03X_H]={0};//标记矩阵
float QQQQ;
float count19;
float count20;
float count21;

float screen_x;
float screen_y;
int xx=0;//记录X坐标
int yy=0;//记录有坐标
void Set_image_twovalues(uint8 value)   ////设置二值化（阈值）
{
    for(uint8 i=0;i<MT9V03X_H;i++)
    {

        for(uint8 j=0;j<MT9V03X_W;j++)
        {

                 // 二值化处理
                  mt9v03x_image_BandW[i][j] = (mt9v03x_image[i][j] >= value) ? 1 : 0;
}
}
                  erosion_operation(mt9v03x_image_BandW);//腐蚀一下
                 // 转换为显示格式
      for(uint8 l=0;l<MT9V03X_H;l++)
        {
           for(uint8 k=0;k<MT9V03X_W;k++)
              {
                  display_image[l][k] = mt9v03x_image_BandW[l][k] * 255;
              }
        }
}

volatile uint8_t dotcnt=0;
volatile uint8_t dotlie[188];//很重要，这个是在屏幕中找白点的列，调整不好会快到白点就会失控
volatile uint8_t dothang[120];//同上
volatile short sumlie=0,sumhang=0;


void Seek_Beacon ()//找灯程序
{
  uint8_t tm=0;
  uint8_t nr=0; //行
  uint8_t nc=0; //列

  dotcnt=0;
  for (nr = 1; nr <MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc <MT9V03X_W - 1; nc++)
    {
      // 188*120分辨率下，有两个连续白点，则认为是可能的信标灯
      if ((mt9v03x_image_BandW[nr - 1][nc] +mt9v03x_image_BandW[nr + 1][nc] +mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] > 1))
      {
        dothang[dotcnt]=nr;     // 记录所有的白点水平方向方位，也就是左右偏差
        dotlie[dotcnt++]=nc;    // 记录所有的白点水垂直向方位，也就是远近
      }
    }
  }
  if(dotcnt) // 发现有白斑
  {
    sumlie=0,sumhang=0;         // 清除上次的结果
    for(tm=0;tm<dotcnt;tm++)
    {
      sumlie+=dotlie[tm];     // 所有白点左右偏差值求和
      sumhang+=dothang[tm];   // 所有白点上下偏差值求和，暂时没用到
    }
    sumlie =sumlie/dotcnt;      // 灯的左右中心点
    sumhang=sumhang/dotcnt;     // 灯的远近中心点，暂时没用到
  }

  ips200_draw_line(0,sumhang,188,sumhang,RGB565_RED);
  ips200_draw_line(sumlie,0,sumlie,120,RGB565_RED);

  int8_t comparative=MT9V03X_W/2-sumlie;
  //ips200_show_int(150,150,sumhang,8);
  ips200_show_int(150,150,comparative,8);
  return;
}



int16_t OFFSET0;    // 最远处，赛道中心值综合偏移量
int16_t OFFSET1;    // 第二格
int16_t OFFSET2;    // 最近，第三格
int area_map[100]={0};


/*****************************************************************************
*函数名称：void SearchNeighbor_4(unsigned char *bitmap, unsigned char *labelmap,uint16 labelIndex,uint16 pixelIndex, Queue *queue)
*功能说明：四邻域生长法（从一个种子点（pixelIndex）出发，检查其上下左右四个相邻像素，将满足条件的像素加入当前连通域，并用队列管理待检查的像素）
*       用于在二值图像（黑白图像）中找出所有连通的白色区域（目标），并计算每个连通域的面积、中心位置和亮度总和
*参数说明：bitmap:代标记图像  labelmap:标记矩阵
*函数返回：无
*修改时间：2022年4月3日
*备    注：
******************************************************************************/
uint16_t area;//记录连通域的可连通点个数
int Connected_x[100];// 记录每个连通域的x坐标总和
int Connected_y[100];// 记录每个连通域的y坐标总和
short Connected_xmin[100];//最小横坐标
short Connected_ymin[100];//最小纵坐标
short Connected_xmax[100];//最大横坐标
short Connected_ymax[100];//最大纵坐标
int32_t Connected_light[100];//累加连通域内所有像素的亮度值，用于计算平均亮度。

//四邻域生长
static int NeighborDirection[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};  //四邻域的偏移量（右、下、左、上）
void SearchNeighbor_4  (unsigned char *bitmap, // 输入二值图像（0或1）
                        unsigned char *labelmap, // 输出标记矩阵（记录每个像素属于哪个连通域）
                        uint8 labelIndex,// 当前连通域的标签值，（改），uint16改为uint8
                        int pixelIndex,  // 当前种子像素的一维索引
                        Queue *queue,// 队列，用于管理待处理的像素
                        int x,int y,// 当前种子像素的二维坐标（x=列，y=行）
                        unsigned char *lightmap) // 亮度图（用于计算连通域的平均亮度）
{
  int searchIndex,i,length;
  int qx,qy;
  labelmap[pixelIndex]=labelIndex;//将当前像素 pixelIndex 标记为属于连通域 labelIndex
  length=MT9V03X_W*MT9V03X_H;//将二维数组转化为一维数组的总数据个数
  
  //area++;  // 累加当前点
  //Connected_x[labelIndex] += x;
  //Connected_y[labelIndex] += y;
  //Connected_light[labelIndex] += lightmap[pixelIndex];
    
  for(i=0;i<4;i++)//二维坐标 (x,y) 转一维索引公式：index = y * width + x，邻域像素的一维索引：searchIndex = pixelIndex + Δy * width + Δx（通过 NeighborDirection 计算偏移）
  {
   searchIndex=pixelIndex+NeighborDirection[i][0]*MT9V03X_W+NeighborDirection[i][1];//二维数组转化为一维数组

   if(searchIndex >= 0 && searchIndex <length && bitmap[searchIndex] == 1 && labelmap[searchIndex] == 0)//检测有无超图像范围,将8领域中未遍历的白点入队
   {
     
     labelmap[searchIndex]=labelIndex;//将区域标记数赋给模板中的这个像素
     area++;
     

     
     //qy=y;
     //qx=(int)((searchIndex-y)/(MT9V03X_W));
     qy = searchIndex / MT9V03X_W;  // 计算行坐标
     qx = searchIndex % MT9V03X_W;  // 计算列坐标
     
     /*if(qy>Connected_ymax[labelIndex])
          Connected_ymax[labelIndex]=qy;
     if(qy<Connected_ymin[labelIndex])
          Connected_ymin[labelIndex]=qy;
     if(qx>Connected_xmax[labelIndex])
          Connected_xmax[labelIndex]=qx;
     if(qx<Connected_xmin[labelIndex])
          Connected_xmin[labelIndex]=qx;
     */
     Connected_x[labelIndex]+=qx;
     Connected_y[labelIndex]+=qy;
     //if(Connected_x[labelIndex]>=32767)
       //Connected_x[labelIndex]=0;

     Connected_light[labelIndex]+=lightmap[searchIndex];

     //防止数据溢出导致死机
     if(area>=65535)
     {
       area=65535;
       break;//break
     }
     PushQueue(queue,searchIndex);//将四邻域中未被遍历过的白点入队 记下坐标为数据域
   }
  }
}


/*****************************************************************************
*函数名称：void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap)
*功能说明：连通域计算,无信标灯判断
*       用于在二值图像中识别并标记所有独立的连通区域（目标），同时计算每个区域的面积、中心坐标和平均亮度
*参数说明：bitmap:代标记图像  labelmap:标记矩阵
*函数返回：返回总的区域计数
*修改时间：2022年4月3日
*备    注：无
*使用说明：只需调用输出数组即可，i从2开始
          area_map[i]为每个独立区域的面积
          Connected_x[i],Connected_y[i],为每个独立区域的坐标
          Connected_light[i]，为每个区域的亮度平均值
******************************************************************************/
//设计的连通域最多十一个区域
uint16_t areamax=0;//最大范围连通域的可连通点个数
uint8_t labelmax=0,//最大范围连通域标号
        labeldel[100]={0}, //可连通点小于3的连通区域标号
        labelcount=0;  //总的连通域个数

//bitmap:代标记图像  labelmap:标记矩阵
void ConnectedComponentLabeling(unsigned char *bitmap, unsigned char * labelmap)
{
  uint8_t i,j,labelIndex=1,del=0,x=1;
  int index,popIndex;
  Queue *queue = NULL;
  queue = (Queue*)malloc(sizeof(Queue));
  queue->first = NULL;
  queue->last = NULL;
  memset(labelmap,0,MT9V03X_W*MT9V03X_H);//清空标记矩阵,每次清0很重要
  memset(labeldel,0,100);//清空通点小于3的连通区域标号
  memset(area_map,0,100);
  //memset(Connected_x,0,100);
  //memset(Connected_y,0,100);
  memset(Connected_x, 0, sizeof(Connected_x));
  memset(Connected_y, 0, sizeof(Connected_y));
  memset(Connected_light,0,100);
  areamax=0;
  labelcount=0;
  labelmax=1;
  int flag_1;
  int last_labelIndex=0;

  for(i = 1; i <=MT9V03X_H-1 ; i++)
  {
    for(j = 1; j <=MT9V03X_W-1 ; j++)
    {
      index=i*(MT9V03X_W)+j;//索引的点 二维数组用一维数组存储 索引的方式
      if(bitmap[index]==1 && labelmap[index]==0)//白点且未被入队
      {
        /*if(labelIndex>=8)
        {
        break;
        }*/
        labelIndex++;//区域计数
        SearchNeighbor_4(bitmap,labelmap,labelIndex,index,queue,i,j,(unsigned char *)mt9v03x_image);//将其八邻域点入队 并在其模板上记下区域标记
        popIndex=PopQueue(queue);//确定第一个区域生长种子
        while(popIndex>-1)// pop的同时 不断搜索其八邻域是否有新的白点并再入队一直重复  一直pop直到队列中没有结点此时这个区域为独立的
        {
         
          SearchNeighbor_4(bitmap,labelmap,labelIndex,popIndex,queue,i,j,(unsigned char *)mt9v03x_image);//不再有白点与其连续一个领域标记完成
          popIndex = PopQueue(queue);//弹出区域生长种子作为下一次的搜索起始位置,知道不再有区域生长种子,整块区域遍历完成
          
        }
      }
      flag_1=labelIndex - last_labelIndex;
      last_labelIndex = labelIndex;

      if(flag_1==1)
      {
          area_map[labelIndex]=area; //保存连通域的面积

          Connected_x[labelIndex]=(int)(Connected_x[labelIndex]/area);  //连通域重心x
          Connected_y[labelIndex]=(int)(Connected_y[labelIndex]/area);  //连通域重心y
          Connected_light[labelIndex]=(int)(Connected_light[labelIndex]/area);
          
          
          flag_1 = 0;
      }

      if(x!=labelIndex)//不止一个连通区域
      {
        x=labelIndex;//X代表了有几个连通区域
        if(area>areamax)//确定最大范围的连通域
        {
          areamax=area;
          labelmax=labelIndex;
        }

        if(area<=1)//     噪音点判断,可有可无
        {
          labeldel[del]=labelIndex;
          del++;
          if(del>=9) del=9;
        }

        area=0;
      }
    }
  }
  free(queue);//一幅图遍历完释放队列空间

  //返回总的区域计数
  labelcount=labelIndex-1;

}

/********************************************
设置权重判断
        面积--高度      亮度      形状
量级  0~1500   0~120   0~255       0~
面积->小  高度->高  可信度->高
面积->大  高度->低  可信度->高

亮度->高->可信度高
形状->高->可信度高
*********************************************/
int quanzhi_weight[100];
void quanzhong_light_panduan(void)
{
    float zz_return;
    memset(quanzhi_weight,0,100);
    for(int i=2;i<=labelcount+2;i++)
    {
        zz_return = Square_detection(Connected_x[i],Connected_y[i]);
        if(zz_return<0.8)//过于竖长
        {
          
            quanzhi_weight[i]=0;
            continue;
        }
        if(((Connected_x[i]<30&&(Connected_y[i]<30||Connected_y[i]>100)) || (Connected_x[i]>130&&(Connected_y[i]<30||Connected_y[i]>100)))&&area_map[i]>30)//远处太大
        {
            quanzhi_weight[i]=0;
            continue;
        }
        if((Connected_x[i]>54&&Connected_x[i]<74&&Connected_y[i]>30&&Connected_y[i]<100 )&& area_map[i]<10)//近处太小
        {
            quanzhi_weight[i]=0;
            continue;
        }
        //权重值越大，是信标灯的概率越大
        //120和1500为当前使用镜头下的参数，不同镜头需要自行修改
        //120为图像上下高度，1500为图像默认情况下可能出现的最大面积
        quanzhi_weight[i]= (int) (((1-(abs((int)Connected_x[i]-94)*1.0/94))+(1-(abs((int)Connected_y[i]-60)*1.0/60)))*10+(area_map[i]*1.0/10000)+Connected_light[i]+zz_return*10);
 
    }

}
int center_x ; 
int center_y ;
//寻找最大权重值
int find_max(void)
{
    int wei=2;
    int maxx=quanzhi_weight[2];
    for(int i=3;i<=labelcount+2;i++)
    {
        if(quanzhi_weight[i]>maxx)
        {
            maxx = quanzhi_weight[i];
            wei=i;
        }
    }
     //画线
          
          float screen_x = Connected_x[wei] * 128.0f / MT9V03X_W;
          float screen_y = Connected_y[wei] * 81.0f / MT9V03X_H;
          
          center_x =Connected_x[wei]; 
          center_y =Connected_y[wei];
          //xx = (int)screen_x;
          //yy = (int)screen_y;
          
           //映射到目标分辨率 (128×81)
          //float screen_x = center_x[wei] * (128.0f / MT9V03X_W);
          //float screen_y = center_y[wei] * (81.0f / MT9V03X_H);
          //float screen_x = center_x[wei] * (127.0f / (MT9V03X_W - 1));  // 映射到0-127
          //float screen_y = center_y[wei] * (81.0f / (MT9V03X_H - 1));   // 映射到0-81
          //xx = (int)screen_x;
          //yy = (int)screen_y;
           xx = (int)screen_x;  // 四舍五入
           yy = (int)screen_y;
          
          //xx=(int)center_x[wei];
          //yy=(int)center_y[wei];
          
          if(xx>=127)xx=127;
          if(xx<=0)xx=0;
          if(yy>=81)yy=81;
          if(yy<=0)yy=0;
    return wei;
}

float KKKK;




//计算最大权重目标位置
/*void quanzhong_light(void)
{
    dotcnt=0;
    int zxc=find_max();
    int16_t comparative=MT9V03X_W/2-sumlie;
                 //短边   长边
    

    if_look_light = 0;

    if(quanzhi_weight[zxc]!=0)
    {
        QQQQ = 1;//清零转向值标志

        if_look_light = 1;//看到灯了

        keep_going = 70;//手动延时
        eye_look_light = 1;
        sumlie = Connected_y[zxc];
        sumhang = Connected_x[zxc];
        dotcnt++;
        //move_k1 = 0.015;

        target_turn = comparative ;
if(sumhang >=80 )
{

    target_turn = 0 ;

}



        if (count19 ==1 )
            {
                count20 = 20 ;
                count19 = 0 ;
            }
            else
            {
                count20 = 0 ;
            }

        if(target_turn <= 7 + count20 && target_turn >= -7 - count20 )
        {

            target_turn = 0;
            count19 = 1;

        }

        ips200_show_int(150,130,target_turn,8);
        if(target_turn  <= 25 && target_turn  >= -25 )//&& double_speed() <= 700)
            {


              target_turn = 0;
              count21 ++ ;
              if(count21 >= 80)
              {

              set_speed = 0;

              }


            }
            else
            {
                count21 = 0;
                set_speed = 0;
            }

    }

    else
    {
      //  move_k1 = 0.015;
        keep_going-- ;
        if(keep_going <= 10)//到时间了停止
        {
           // qianhou_zhongzhi = 1.54;
            set_speed = 0;
            keep_going = 5;
            if_turn = 1 ;
        }
        else if(keep_going > 10)//短时间内继续前进
        {
            //qianhou_zhongzhi = 3.0;
            set_speed = 0 ;
            if_turn = 0 ;
        }

        ips200_show_int(150,130,target_turn,8);

        if (QQQQ == 1)  //防止和中断抢赋值，看不到灯只清零一次,1,一开始就看见灯，然后看不见，由1变0
        {               //2,一开始看不见见灯，然后看见，由0变1
        target_turn = 0 ;
        QQQQ = 0 ;
        }
        eye_look_light = 2;//表示看不到灯
    }
}
*/

/*---------------------------------------------------------------
【函    数】Bin_Image_Filter
【功    能】过滤噪点，如果一个黑色像素被较多的白色像素包围（超过两个），则将其改为白色，这可能是为了消除孤立的黑色噪声点。
         如果一个白色像素被较少的白色像素包围（少于两个），则将其改为黑色，这可能是为了消除孤立的白色噪声点。
【参    数】无
【返 回 值】无
【注意事项】
----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
  int  nr; //行
  int  nc; //列

  for (nr = 1; nr < MT9V03X_H - 1; nr++)
  {
    for (nc = 1; nc < MT9V03X_W- 1; nc = nc + 1)
    {
      if ((mt9v03x_image_BandW[nr][nc] == 0)
          && (mt9v03x_image_BandW[nr - 1][nc] + mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] > 2))
      {
          mt9v03x_image_BandW[nr][nc] = 1;
      }
      else if ((mt9v03x_image_BandW[nr][nc] == 1)
               && (mt9v03x_image_BandW[nr - 1][nc] + mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr][nc + 1] + mt9v03x_image_BandW[nr][nc - 1] < 2))
      {
          mt9v03x_image_BandW[nr][nc] = 0;
      }
    }
  }
}

/*************************************************************************
*  函数名称：void eye_show(void)
*  功能说明：可视化识别出的信标灯的位置，并显示出十字准星
*  参数说明：无
*  函数返回：无
*  修改时间：2022年3月31日
*  备    注：  Bin_Image[LCDH][LCDW]; 二值化后图像数组
*************************************************************************/
void eye_show (void)//显示十字
{
  for(int i = 1 ;i<MT9V03X_H-1;i++)
  {
      mt9v03x_image_BandW[i][sumlie] = 1;
  }
  for(int i = 1 ;i<MT9V03X_W-1;i++)
  {
      mt9v03x_image_BandW[sumhang][i] = 1;
  }
    return;
}

/**************************************************************************
*                                                                          *
*  函数名称：Seek_Road(void)                                                   *
*  功能说明：寻找白色区域偏差值                                                      *
*  参数说明：无                                                            *
*  函数返回：值的大小                                                      *
*  修改时间：2017-07-16                                                    *
*  备    注：以中间为0，左侧减一，右侧加一，数值代表1的面积                            *
*            计算区域从第一行开始到倒数第二行结束。                                 *
*            如果面积为负数，数值越大说明越偏左边；                                  *
*            如果面积为正数，数值越大说明越偏右边。                                  *
**************************************************************************/
void Seek_Road (void)
{
    int16_t nr; //行
    int16_t nc; //列
    int16_t temp = 0; //临时数值
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp = 0;
    int16 MAX_COL=MT9V03X_W;   //自己写的，有可能有错
    for (nr = 8; nr < 40; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET0 = temp;
    temp = 0;
    for (nr = 40; nr < 80; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET1 = temp;
    temp = 0;
    for (nr = 80; nr < 110; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (mt9v03x_image_BandW[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET2 = temp;
    return;
}

/*****************************************************************************
*函数名称：void PushQueue(Queue *queue,int data)
*功能说明：入队
*参数说明：无
*函数返回：无
*修改时间：2022年4月3日
*备    注：入队,本质上是对队尾的处理,开辟内存加入队尾
******************************************************************************/
void PushQueue(Queue *queue,int data)   //向队列queue中插入一个值为data的新元素
{
    QNode *p=NULL;//*p是队列节点指针 QNode是对他属于节点属性的说明
    p=(QNode*)malloc(sizeof(QNode));//创造一个队列结点(p等于这个节点的地址)
    p->data=data;
    if(queue->first==NULL)//若此时未指定队首和队尾 则这位第一个结点 将这个结点定为队首和队尾  （空队）
    {
     queue->first=p;
     queue->last=p;
     p->next=NULL;
    }
    else  //（不是空队）
    {
     p->next=NULL;//将新的结点加在队尾为空--
     queue->last->next=p;//旧的队列结尾的下一个是P
     queue->last=p;//新的队列结尾
    }
}
//
/*****************************************************************************
*函数名称：int PopQueue(Queue *queue)
*功能说明：出队
*参数说明：无
*函数返回：无
*修改时间：2022年4月3日
*备    注：出队,本质上是对队首的处理,释放内存弹出队首
******************************************************************************/
int PopQueue(Queue *queue) //从队列queue中移除队首元素，并返回它的值
{
    QNode *p=NULL;
    int data;
    if(queue->first==NULL)//队空则pop失败返回-1
    {
     return -1;
    }
     p=queue->first;//指针P指向队首
     data=p->data;//提取队首的数据
    if(queue->first->next == NULL)//若为最后一个结点
    {
     queue->first=NULL;
     queue->last=NULL;
    }
    else
    {
     queue->first=p->next;
    }
    free(p);//将队首结点弹出并释放空间
    return data;
}


/*************************************************************************
*  函数名称：float Square_detection (int x,int y)
*  功能说明：对目标进行简单的长宽判断
*  参数说明：无
*  函数返回：无
*  修改时间：2022年7月11日
*  备    注：  mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W]; 二值化后图像数组
*************************************************************************/
uint8_t shang=0,xia=0,zuo=0,you=0;
float Square_detection(int16_t x,int16_t y)
{
   // uint8_t tm=0;
    int16_t nr=x; //行
    int16_t nc=y; //列
    shang=0,xia=0,zuo=0,you=0;  //上下左右
    dotcnt=0;

if(nr>3&&nr<116&&nc>3&&nc<184){         //防止逃出图片范围，变成野指针
    if((mt9v03x_image_BandW[nr][nc] == 1) || (mt9v03x_image_BandW[nr + 1][nc] + mt9v03x_image_BandW[nr + 3][nc]+ mt9v03x_image_BandW[nr - 1][nc] +mt9v03x_image_BandW[nr - 3][nc] +
      mt9v03x_image_BandW[nr][nc + 1] +mt9v03x_image_BandW[nr][nc + 3]+mt9v03x_image_BandW[nr][nc -3] +mt9v03x_image_BandW[nr][nc - 1] >= 4) )
    {
        while (mt9v03x_image_BandW[nr - shang][nc] ==1)
        {
            shang++;
        }
        while (mt9v03x_image_BandW[nr + xia][nc] == 1)
        {
            xia++;
        }
        while (mt9v03x_image_BandW[nr][nc - zuo] == 1)
        {
            zuo++;
        }
        while (mt9v03x_image_BandW[nr][nc + you] == 1)
        {
            you++;
        }
        //求长宽比例
        return (abs(zuo+you)/2.0) / (abs(shang+xia)/2.0);

    }
    else return (0);
}
else return (0);
}

