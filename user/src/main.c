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
int main(void)
{
    //     disableInterrupts();
    // IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    // //用户在此处调用各种初始化函数等

    // //等待所有核心初始化完毕
    // // IfxCpu_emitEvent(&g_cpuSyncEvent);
    // // IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    // enableInterrupts();
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "Engine started.\r\n");
    Initall();                     // 初始化所有
    DATA_INIT();                   // 初始化参数
    clock_init(SYSTEM_CLOCK_120M); // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                  // 初始化默认 Debug UART

    ips200_init(IPS200_TYPE_PARALLEL8);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    system_delay_ms(1000); // 务必延时
    key_init(10);
    // 此处编写用户代码 例如外设初始化代码等
    // sdcard_read();

    // adaptiveThreshold_1(mt9v03x_image, outimage, MT9V03X_W, MT9V03X_H, 7, 8);
    // adaptiveThreshold_2(mt9v03x_image, outimage, 7, 12);
    // ImageBinary(image_read_buffer, outimage, MT9V03X_W, MT9V03X_H);
    // (outimage, outimage1);

    // int i=0;

    Menu_Switch();

    // timer_start(TIM_2);
    // mt9v03x_get_version();
    // imu660ra_get_acc();  // 获取 IMU660RA 的加速度测量数值
    // imu660ra_get_gyro(); // 获取 IMU660RA 的角速度测量数值

    // Key_Switch();
    // button();

    //         if (mt9v03x_finish_flag) {
    //             // timer_start(TIM_2);
    //             // adaptiveThreshold_2();
    //             // Full_Inverse_Perspective();
    //             wusuowei(mt9v03x_image, &g_Border, &g_TrackType);
    //             FindCorner(&g_Border, &g_TrackType);
    //             GetAimingDist(&g_Border, &g_LineError, &g_TrackType);
    //             PurePursuit(&g_Border, &g_LineError, &g_TrackType);

    //             // ips200_displayimage03x(mt9v03x_image[0], IMGW, IMGH);
    //             // ips200_show_float(10, 130, g_LineError.m_f32LeftBorderKappa, 4, 4);
    //             // ips200_show_float(10, 150, g_LineError.m_f32RightBorderKappa, 4, 4);
    //             // DrawCenter(&g_Border);

    //             // 显示陀螺仪数据
    //             // ips200_show_float(10, 80, imu660ra_acc_x, 4, 4);
    //             // ips200_show_float(10, 100, imu660ra_gyro_x, 4, 4);
    //             // ips200_show_float(10, 120, imu660ra_acc_y, 4, 4);
    //             // ips200_show_float(10, 140, imu660ra_gyro_y, 4, 4);
    //             // ips200_show_float(10, 160, imu660ra_acc_z, 4, 4);
    //             // ips200_show_float(10, 180, imu660ra_gyro_z, 4, 4);

    //             // ips200_show_float(10, 200, yaw_angle, 4, 4);
    //             // ips200_show_float(10, 220, g_z, 4, 4);

    //             // timer_stop(TIM_2);
    //             // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> POCEESS TIME ==%d", timer_get(TIM_2));
    //             // timer_clear(TIM_2);

    //             // DrawBoarder(&g_Border);

    //             Control();

    // #if 0
    //         wireless_uart_send_buff(virsco_data, 100);
    //         virtual_oscilloscope_data_conversion(encoder_2, Motor_Right.result, encoder_1, Motor_Left.result);
    //         system_delay_ms(100);
    // #endif

    //             mt9v03x_finish_flag = 0;
    //         }

    // // 检查拐点
    // FindCorner(&g_Border, &g_TrackType);
    // // 获取预瞄距离
    // GetAimingDist(&g_Border, &g_LineError, &g_TrackType);
    // // 纯跟踪计算赛道曲率
    // PurePursuit(&g_Border, &g_LineError, &g_TrackType);

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
    //     // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\r\n LOG -> rx=%d.", g_Border.m_LPntout[i].m_i16x);
    //     // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\r\n LOG -> ry=%d.", g_Border.m_LPntout[i].m_i16y);
    //     // unsigned char length;
    //     // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> bx=%s.", out_float(g_Border.m_LCPnt[i].m_i16x, 4, &length));
    //     // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> djx=%s.", out_float(g_Border.m_LCPnt[i].m_i16y, 4, &length));
    // }
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\r\n LOG -> inv=%d.", Inv_x[3049]);
    // timer_stop(TIM_2);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> POCEESS TIME ==%d", timer_get(TIM_2));
    // timer_clear(TIM_2);
    // system_delay_ms(20);

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

    // while (1) {

    //     // gpio_toggle_level(LED1);
    //     // system_delay_ms(1000);

    //     // Key_Switch();

    // }
}