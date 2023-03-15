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

// uint8 virsco_data[10];
int main(void)
{
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

    /***************测时间******************/
    // timer_start(TIM_2);
    // timer_stop(TIM_2);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> POCEESS TIME ==%d", timer_get(TIM_2));
    // timer_clear(TIM_2);
    /************************************/

    while (1) {
        // timer_start(TIM_2);

        if (mt9v03x_finish_flag) {
            if (Protect_Frame > 0) Protect_Frame--;
            /**************未使用函数****************/
            // adaptiveThreshold_2();
            // Full_Inverse_Perspective();
            // imu660ra_get_acc();  // 获取 IMU660RA 的加速度测量数值
            // imu660ra_get_gyro(); // 获取 IMU660RA 的角速度测量数值
            // Key_Switch();
            //          if (Img_Open_falg) {
            //     ips200_displayimage03x(mt9v03x_image[0], 188, 120);
            // }
            /***************************************/

            Out_Protect(mt9v03x_image);
            wusuowei(mt9v03x_image, &g_Border, &g_TrackType);
            FindCorner(&g_Border, &g_TrackType);

            /**************************************************************************************元素判断函数群**********************************************************************/

            // 十字
            if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE && Protect_Frame == 0) { Check_Cross(mt9v03x_image, &g_Border, &g_TrackType); }
            // 中入十字
            if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE && Protect_Frame == 0) { Check_MIDCross(mt9v03x_image, &g_Border, &g_TrackType); }
            // 右斜入三岔，三个直角拐点
            if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE && Protect_Frame == 0) { RightThreeCornerCross(mt9v03x_image, &g_Border, &g_TrackType); }
            // 左斜入三岔，三个直角拐点
            if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE && Protect_Frame == 0) { LeftThreeCornerCross(mt9v03x_image, &g_Border, &g_TrackType); };
            // 右环岛
            if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8CrossFlag == CROSS_NONE && g_TrackType.m_u8RightSideCrossFlag == CROSS_NONE && g_TrackType.m_u8LeftSideCrossFlag == CROSS_NONE && Protect_Frame == 0) { Check_RightRoundabout(&g_Border, &g_TrackType); }
            // 左环岛
            if (g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8CrossFlag == CROSS_NONE && g_TrackType.m_u8RightSideCrossFlag == CROSS_NONE && g_TrackType.m_u8LeftSideCrossFlag == CROSS_NONE && Protect_Frame == 0) { Check_LeftRoundabout(&g_Border, &g_TrackType); }
            // 中入左环岛，用于错过环岛一阶段后直接进入环岛二阶段
            if (g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8CrossFlag == CROSS_NONE && g_TrackType.m_u8RightSideCrossFlag == CROSS_NONE && g_TrackType.m_u8LeftSideCrossFlag == CROSS_NONE && Protect_Frame == 0) { Check_MIDLeftRoundabout(mt9v03x_image, &g_Border, &g_TrackType, &g_LineError); }
            // 中入右环岛，用于错过环岛一阶段后直接进入环岛二阶段
            if (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE && g_TrackType.m_u8CrossFlag == CROSS_NONE && g_TrackType.m_u8RightSideCrossFlag == CROSS_NONE && g_TrackType.m_u8LeftSideCrossFlag == CROSS_NONE && Protect_Frame == 0) { Check_MIDRightRoundabout(mt9v03x_image, &g_Border, &g_TrackType, &g_LineError); }

            /********************************************************************************************************************************************************************/

            /*******************************ips显示区***********************/
            ips200_displayimage03x(mt9v03x_image[0], IMGW, IMGH);//显示原图像
            DrawBoarder(&g_Border); // 原边线
            // DrawCenter(&g_Border);      // 逆透视后中线
            DrawBoarderInvp(&g_Border); // 逆透视后边线
            DrawRemoteLine(&g_Border);  // 远端边线数组
            ips200_draw_line(0, 120 - g_LineError.m_f32LeftBorderAimingMin / SampleDist, 188, 120 - g_LineError.m_f32LeftBorderAimingMin / SampleDist, RGB565_RED);
            ips200_draw_line(0, 120 - g_LineError.m_f32LeftBorderAimingMax / SampleDist, 188, 120 - g_LineError.m_f32LeftBorderAimingMax / SampleDist, RGB565_RED);
            ips200_show_int(30, 130, g_TrackType.m_u8RightRoundaboutFlag, 4); // 环岛标志位
            ips200_show_int(30, 150, g_TrackType.m_u8CrossFlag, 4);           // 十字标志位
            ips200_show_int(30, 170, Protect_Frame, 4);                       // 不造什么标志位（运行完一个元素则赋值）
            ips200_show_int(30, 190, g_Border.LL_CornerPos, 4);               // 左边线L角点位置
            ips200_show_int(30, 210, g_Border.RL_CornerPos, 4);               // 右边线L角点位置
            ips200_show_int(50, 230, g_Border.RL_CornerPosRemote, 4);
            ips200_show_int(50, 250, g_Border.LL_CornerPosRemote, 4);
            ips200_show_int(90, 270, g_Border.m_i16LPointCntRS, 4);
            ips200_show_int(90, 290, g_Border.m_i16RPointCntRS, 4);
            ips200_show_int(130, 130, g_TrackType.m_u8ShortRightLineStraightFlag, 4);
            ips200_show_int(130, 150, g_TrackType.m_u8ShortLeftLineStraightFlag, 4);

            ips200_show_string(0, 130, "HD:");
            ips200_show_string(0, 150, "SZ:");
            ips200_show_string(0, 170, "PF:");
            ips200_show_string(0, 190, "LL:");
            ips200_show_string(0, 210, "RL:");
            ips200_show_string(0, 230, "LLfar:");
            ips200_show_string(0, 250, "RLfar:");
            ips200_show_string(0, 270, "Lline size:");
            ips200_show_string(0, 290, "Rline size:");
            ips200_show_string(95, 130, "RDZ:");
            ips200_show_string(95, 150, "LDZ:");
            // ips200_show_int(10, 190, g_TrackType.m_u8CrossFlag, 4);
            // ips200_show_int(10, 130, g_TrackType.Outframe, 4);
            // ips200_show_float(10, 200, yaw_angle, 4, 4);
            // ips200_show_float(10, 150, g_LineError.m_f32RightBorderKappa, 4, 4);
            /****************************************************************/

            // 获取预瞄距离
            GetAimingDist(&g_Border, &g_LineError, &g_TrackType);
            // 纯跟踪计算赛道曲率
            PurePursuit(&g_Border, &g_LineError, &g_TrackType);

            Control();

#if 0
        wireless_uart_send_buff(virsco_data, 100);
        virtual_oscilloscope_data_conversion(encoder_2, Motor_Right.result, encoder_1, Motor_Left.result);
        system_delay_ms(100);
#endif

            mt9v03x_finish_flag = 0;
        }
    }
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> Chang_Lpoint success.");
}