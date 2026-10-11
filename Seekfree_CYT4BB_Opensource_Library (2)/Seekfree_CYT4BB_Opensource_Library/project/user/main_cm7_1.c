/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "find.h"
#include "globals.h"
#include "image.h"
#include "car_control.h"
#include "kaerman.h"
#include "quaternion.h"
#include "ztjx.h"
#include "my_encoder.h"

#define ENCODER_QUADDEC1                (TC_CH58_ENCODER)                       // 带方向编码器对应使用的编码器接口       
#define ENCODER_QUADDEC_A1              (TC_CH58_ENCODER_CH1_P17_3)             // A 对应的引脚                      
#define ENCODER_QUADDEC_B1              (TC_CH58_ENCODER_CH2_P17_4)             // B 对应的引脚                     

#define ENCODER_QUADDEC2                (TC_CH27_ENCODER)                       // 带方向编码器对应使用的编码器接口   
#define ENCODER_QUADDEC_A2              (TC_CH27_ENCODER_CH1_P19_2)             // A 对应的引脚                  
#define ENCODER_QUADDEC_B2              (TC_CH27_ENCODER_CH2_P19_3)             // B 对应的引脚                       

#define ENCODER_QUADDEC3                (TC_CH07_ENCODER)                       // 带方向编码器对应使用的编码器接口  
#define ENCODER_QUADDEC_A3              (TC_CH07_ENCODER_CH1_P07_6)             // A 对应的引脚                 
#define ENCODER_QUADDEC_B3              (TC_CH07_ENCODER_CH2_P07_7)             // B 对应的引脚                       

#define ENCODER_QUADDEC4                (TC_CH20_ENCODER)                       // 带方向编码器对应使用的编码器接口
#define ENCODER_QUADDEC_A4              (TC_CH20_ENCODER_CH1_P08_1)             // A 对应的引脚
#define ENCODER_QUADDEC_B4              (TC_CH20_ENCODER_CH2_P08_2)             // B 对应的引脚
static int len =0;

// ******************************* 代码区域 *******************************
/*#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P00_0
*/
uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

uint8 by_fly[5];
fifo_struct uart_data_fifo;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化
    uart_init(UART_1, 9600, UART1_TX_P04_1, UART1_RX_P04_0);    
    uart_rx_interrupt(UART_1, 1);  
    kaerman_b_Init(&gyro_x,2.0f,2.0f,80.0f,80.0f,0.002f,0.002f);
    kaerman_b_Init(&gyro_y,1.0f,1.0f,80.0f,80.0f,0.002f,0.002f);
    kaerman_b_Init(&gyro_z,1.0f,1.0f,80.0f,80.0f,0.002f,0.002f);
    // 此处编写用户代码 例如外设初始化代码等
    quaternion_Init(&sc,&s_angle,&s_shu,1.2,0.001,0.002f);//pi,角度，四元数，kp,kd,中断时间,2.0,0.001,0.05f
    imu660rb_init();
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);

    //888888888888888
    //car_Init(0.0,0.0,0.0,0.0,0.0,0.0,  0.0,0.0,0.0);
    car_Init(2.5,0.0,0.12,  2.0,0.0,0.12,  1.5,0.0,0.6);
    //car_Init(2.5,0.0,0.25,  2.5,0.0,0.25,  3.5,0.0,0.35);
    
    //my_encoder_Init(1.0,0.0,0.8, 1.0,0.0,0.8, 0.6,0.0,0.8, 1.2,0.0,0.8);//1.0,0.0,0.5
    
    my_encoder_Init(0.8,0.0,0.6, 0.6,0.0,0.5, 0.6,0.0,0.5, 0.8,0.0,0.6);//1.0,0.0,0.5
    
    //88888888888888888
    encoder_quad_init(ENCODER_QUADDEC1, ENCODER_QUADDEC_A1, ENCODER_QUADDEC_B1);// 初始化编码器模块与引脚 带正交编码器模式
    encoder_quad_init(ENCODER_QUADDEC2, ENCODER_QUADDEC_A2, ENCODER_QUADDEC_B2);// 初始化编码器模块与引脚 带正交编码器模式
    encoder_quad_init(ENCODER_QUADDEC3, ENCODER_QUADDEC_A3, ENCODER_QUADDEC_B3);// 初始化编码器模块与引脚 带正交编码器模式
    encoder_quad_init(ENCODER_QUADDEC4, ENCODER_QUADDEC_A4, ENCODER_QUADDEC_B4);// 初始化编码器模块与引脚 带正交编码器模式
    
    pit_ms_init(PIT_CH0,2);
    pit_enable(PIT_CH0);
    pit_ms_init(PIT_CH1,10);
    pit_enable(PIT_CH1);
    pit_ms_init(PIT_CH2,5);
    pit_enable(PIT_CH2);
    pit_ms_init(PIT_CH10,2);
    pit_enable(PIT_CH10);
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

       // fifo_data_count = fifo_used(&uart_data_fifo);                             // 查看 fifo 是否有数据
       // if(fifo_data_count != 0)                                                // 读取到数据了
       // {
        //   fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
       //     uart_write_string(UART_2, "\r\nUART get data:");                // 输出测试信息
            //uart_write_buffer(UART_2, fifo_get_data, fifo_data_count);      // 将读取到的数据发送出去
            //if(by_fly[0]==0xAA&&by_fly[4]==0x55)
            //{
        system_delay_ms(30);
        rcc_roll = imu660rc_roll;
        rcc_pitch=imu660rc_pitch;
        rcc_yaw = imu660rc_yaw;
        encoder_data_dir[0] = encoder_get_count(ENCODER_QUADDEC1);
        encoder_clear_count(ENCODER_QUADDEC1);
        encoder_data_dir[1] = encoder_get_count(ENCODER_QUADDEC2);
        encoder_clear_count(ENCODER_QUADDEC2);
        encoder_data_dir[2] = encoder_get_count(ENCODER_QUADDEC3);
        encoder_clear_count(ENCODER_QUADDEC3);
        encoder_data_dir[3] = encoder_get_count(ENCODER_QUADDEC4);
        //printf("%d,%d,%d,%d\n",encoder_data_dir[0],encoder_data_dir[1],encoder_data_dir[2],encoder_data_dir[3]);
        encoder_clear_count(ENCODER_QUADDEC4);
        // printf("%f,%f,%f\n",s_angle.agz,imu660rc_pitch,imu660rc_roll -2.0);
        //printf("1");
       //printf("%f,%f,%f\n",imu660rc_roll,imu660rc_pitch,imu660rc_yaw);
         if(by_fly[0]==0xAA&&by_fly[4]==0x55)
         {
           erro_xx=by_fly[1];
           erro_yy=by_fly[2];
           flag_get=by_fly[3];
           //printf("%d,%d,%d\n",by_fly[1],by_fly[2],by_fly[3]);
         }
         else if(by_fly[1]==0xAA&&by_fly[0]==0x55)
         {
           erro_xx=by_fly[2];
           erro_yy=by_fly[3];
           flag_get=by_fly[4];
           //printf("%d,%d,%d\n",by_fly[2],by_fly[3],by_fly[4]);
         }
         else if(by_fly[2]==0xAA&&by_fly[1]==0x55)
         {
           erro_xx=by_fly[3];
           erro_yy=by_fly[4];
           flag_get=by_fly[0];
          //printf("%d,%d,%d\n",by_fly[3],by_fly[4],by_fly[0]);
         }
         else if(by_fly[3]==0xAA&&by_fly[2]==0x55)
         {
           erro_xx=by_fly[4];
           erro_yy=by_fly[0];
           flag_get=by_fly[1];
          //printf("%d,%d,%d\n",by_fly[4],by_fly[0],by_fly[1]);
         }
         else if(by_fly[4]==0xAA&&by_fly[3]==0x55)
         {
           erro_xx=by_fly[0];
           erro_yy=by_fly[1];
           flag_get=by_fly[2];
          //printf("%d,%d,%d\n",by_fly[0],by_fly[1],by_fly[2]);
         }
        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用
// 参数说明       void
// 返回参数       void
// 使用示例       uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);        // 接收数据 while 等待式 不建议在中断使用
  
    //if(uart_query_buffer(UART_2, by_fly))                                  // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    //{
    //    fifo_write_buffer(&uart_data_fifo, by_fly, 5);     
    //}
    //uart_query_buffer(UART_2, by_fly);
  len++;
  if(len>4)len=0;
  uart_query_byte(UART_1, &by_fly[0+len]);
}

// **************************** 代码区域 ****************************
// **************************** 例程常见问题说明 ****************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看串口助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率