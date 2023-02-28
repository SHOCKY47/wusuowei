/*********************************************************************************************************************
 * MM32F327X-G9P Opensourec Library 即（MM32F327X-G9P 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 MM32F327X-G9P 开源库的一部分
 *
 * MM32F327X-G9P 开源库 是免费软件
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
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.37
 * 适用平台          MM32F327X_G9P
 * 店铺链接          https://seekfree.taobao.com/ b.ol
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-08-10        Teternal            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "headfile.h"

uint8 virsco_data[10];

// #define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_1
// #define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
// #define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART1_TX_A9
// #define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART1_RX_A10

// #define UART_PRIORITY           (UART1_IRQn)                                    // 对应串口中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

// uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
// uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

// uint8 get_data = 0;                                                             // 接收数据变量
// uint32 fifo_data_count = 0;                                                     // fifo 数据个数

// fifo_struct uart_data_fifo;

// #define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_1
// #define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
// #define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART1_TX_A9
// #define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART1_RX_A10

// #define UART_PRIORITY           (UART1_IRQn)                                    // 对应串口中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

// uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
// uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

// uint8 get_data = 0;                                                             // 接收数据变量
// uint32 fifo_data_count = 0;                                                     // fifo 数据个数

// fifo_struct uart_data_fifo;

int main(void)
{
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    Initall();   // 初始化所有
    DATA_INIT(); // 初始化参数

    // fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

    // uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    // uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    // interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

    // uart_write_string(UART_INDEX, "UART Text.");                                // 输出测试信息
    // uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    // uart_write_byte(UART_INDEX, '\n');                                          // 输出换行

    // 此处编写用户代码 例如外设初始化代码等
    // sdcard_read();

    // adaptiveThreshold_1(mt9v03x_image, outimage, MT9V03X_W, MT9V03X_H, 7, 8);
    // adaptiveThreshold_2(mt9v03x_image, outimage, 7, 12);
    // ImageBinary(image_read_buffer, outimage, MT9V03X_W, MT9V03X_H);
    // (outimage, outimage1);

    // int i=0;
    // while(1)
    // {
    // timer_start(TIM_2);

    // wusuowei(image_read_buffer, &g_Border, &g_TrackType);

    // 检查拐点
    // FindCorner(&g_Border, &g_TrackType);
    // 十字
    // if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE/*  && g_TrackType.m_u8LeftPRoadFlag == PROAD_NONE && g_TrackType.m_u8RightPRoadFlag == PROAD_NONE&& g_TrackType.m_u8YjunctionFlag == YJUNCTION_NONE */&& Protect_Frame == 0)
    //     Check_Cross(mt9v03x_image, &g_Border, &g_TrackType);
    // // 中入十字
    // if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE/* && g_TrackType.m_u8LeftPRoadFlag == PROAD_NONE && g_TrackType.m_u8RightPRoadFlag == PROAD_NONE && g_TrackType.m_u8YjunctionFlag == YJUNCTION_NONE */&& Protect_Frame == 0)
    //     Check_MIDCross(mt9v03x_image, &g_Border, &g_TrackType);
    // int i = 0;
    // while (++i < 100) {
    //     // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> lx=%d.", g_Border.m_LPnt[i].m_i16x);
    //     // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> ly=%d.", g_Border.m_LPnt[i].m_i16y);
    //     SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\r\n LOG -> rx=%d.", g_Border.m_LCPnt[i].m_i16x);
    //     SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\r\n LOG -> ry=%d.", g_Border.m_RCPnt[i].m_i16y);
    // }
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\r\n LOG -> inv=%d.", Inv_x[3049]);

    // timer_stop(TIM_2);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> POCEESS TIME ==%d", timer_get(TIM_2));
    // timer_clear(TIM_2);

    // }
    // Change_Lpoint(image_read_buffer, 7, outimage, 12);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> Chang_Lpoint success.");
    // findline_lefthand_adaptive(image_read_buffer, 7, 0, &g_Border, 0);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> findline_lefthand_adaptive success.");
    // int i = 1;
    // sdcard_write(mt9v03x_image, sizeof(mt9v03x_image));

    // while (i++ < 120) {
    //     SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> y==%d", g_Border.m_LPnt[i].m_i16y);

    //     SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> x==%d", g_Border.m_LPnt[i].m_i16x);
    // }

    // gpio_set_level(LED1, 0);

    while (1) {

        // gpio_toggle_level(LED1);
        // system_delay_ms(1000);

        // Key_Switch();

        // wireless_uart_send_buff(virsco_data, 100);
        // virtual_oscilloscope_data_conversion(encoder_data_quaddec_2, 0, 0, 0);

        // system_delay_ms(100);
    }
}