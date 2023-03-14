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
 * 文件名称          isr
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.37
 * 适用平台          MM32F327X_G9P
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-08-10        Teternal            first version
 ********************************************************************************************************************/

#include "isr.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM1 的定时器更新中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM1_UP_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM1_UP_IRQHandler(void)
{
    // 此处编写用户代码

    // 此处编写用户代码
    TIM1->SR &= ~TIM1->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM2 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM2_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
    // 此处编写用户代码

    // 此处编写用户代码
    TIM2->SR &= ~TIM2->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM3 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM3_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM3_IRQHandler(void)
{
    // 此处编写用户代码

    // 此处编写用户代码
    TIM3->SR &= ~TIM3->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM4 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM4_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM4_IRQHandler(void)
{
    // 此处编写用户代码

    // 此处编写用户代码
    TIM4->SR &= ~TIM4->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM5 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM5_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM5_IRQHandler(void)
{
    // 此处编写用户代码

    // 此处编写用户代码
    TIM5->SR &= ~TIM5->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM6 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM6_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM6_IRQHandler(void)
{
    // 此处编写用户代码

    if (start_flag == 1) {
        Get_Speed();

        Motor_L_Control(&Motor_Left, &MOTOR, encoder_1);
        Motor_R_Control(&Motor_Right, &MOTOR, encoder_2);

        Back_Wheel_Out(-Motor_Left.result, Motor_Right.result);
    }

    // 此处编写用户代码
    TIM6->SR &= ~TIM6->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM7 的定时器中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM7_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM7_IRQHandler(void)
{
    // 此处编写用户代码

    // gyro_calculate();
    // 此处编写用户代码
    TIM7->SR &= ~TIM7->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TIM8 的定时器更新中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(TIM8_UP_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void TIM8_UP_IRQHandler(void)
{
    tsl1401_collect_pit_handler();
    // 此处编写用户代码

    // 此处编写用户代码
    TIM8->SR &= ~TIM8->SR; // 清空中断状态
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART1_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART1_IRQHandler(void)
{
    if (UART1->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART1->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART1->ISR & 0x00000002) // 串口接收缓冲中断
    {
#if DEBUG_UART_USE_INTERRUPT       // 如果开启 debug 串口中断
        debug_interrupr_handler(); // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                             // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART1->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART2 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART2_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART2_IRQHandler(void)
{
    if (UART2->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART2->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART2->ISR & 0x00000002) // 串口接收缓冲中断
    {
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART2->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART3_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART3_IRQHandler(void)
{
    if (UART3->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART3->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART3->ISR & 0x00000002) // 串口接收缓冲中断
    {
        gps_uart_callback();
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART3->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART4 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART4_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART4_IRQHandler(void)
{
    if (UART4->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART4->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART4->ISR & 0x00000002) // 串口接收缓冲中断
    {
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART4->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART5 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART5_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART5_IRQHandler(void)
{
    if (UART5->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART5->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART5->ISR & 0x00000002) // 串口接收缓冲中断
    {
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART5->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART6 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART6_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART6_IRQHandler(void)
{
    if (UART6->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART6->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART6->ISR & 0x00000002) // 串口接收缓冲中断
    {
        wireless_module_uart_handler();
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART6->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART7 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART7_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART7_IRQHandler(void)
{
    if (UART7->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART7->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART7->ISR & 0x00000002) // 串口接收缓冲中断
    {
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART7->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART8 的串口中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(UART8_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void UART8_IRQHandler(void)
{
    if (UART8->ISR & 0x00000001) // 串口发送缓冲空中断
    {
        // 此处编写用户代码
        // 务必填写数据或者关闭中断 否则会一直触发串口发送中断

        // 此处编写用户代码
        UART8->ICR |= 0x00000001; // 清除中断标志位
    }
    if (UART8->ISR & 0x00000002) // 串口接收缓冲中断
    {
        camera_uart_handler();
        // 此处编写用户代码
        // 务必读取数据或者关闭中断 否则会一直触发串口接收中断

        // 此处编写用户代码
        UART8->ICR |= 0x00000002; // 清除中断标志位
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI0 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI0_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI0_IRQHandler(void)
{
    // 此处编写用户代码 (A0/B0..H0) 引脚触发

    // 此处编写用户代码 (A0/B0..H0) 引脚触发
    EXTI->PR |= (0x00000001 << 0); // 清除 line0 触发标志
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI1 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI1_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI1_IRQHandler(void)
{
    // 此处编写用户代码 (A1/B1..H1) 引脚触发

    // 此处编写用户代码 (A1/B1..H1) 引脚触发
    EXTI->PR |= (0x00000001 << 1); // 清除 line1 触发标志
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI2 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI2_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI2_IRQHandler(void)
{
    // 此处编写用户代码 (A2/B2..H2) 引脚触发

    // 此处编写用户代码 (A2/B2..H2) 引脚触发
    EXTI->PR |= (0x00000001 << 2); // 清除 line2 触发标志
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI3 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI3_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI3_IRQHandler(void)
{
    // 此处编写用户代码 (A3/B3..H3) 引脚触发

    // 此处编写用户代码 (A3/B3..H3) 引脚触发
    EXTI->PR |= (0x00000001 << 3); // 清除 line3 触发标志
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI4 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI4_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI4_IRQHandler(void)
{
    // 此处编写用户代码 (A4/B4..G4) 引脚触发

    // 此处编写用户代码 (A4/B4..G4) 引脚触发
    EXTI->PR |= (0x00000001 << 4); // 清除 line4 触发标志
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI9-5 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI9_5_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (0x00000001 << 5)) // 检测 line5 是否触发
    {
        // 此处编写用户代码 (A5/B5..G5) 引脚触发

        // 此处编写用户代码 (A5/B5..G5) 引脚触发
        EXTI->PR |= (0x00000001 << 5); // 清除 line5 触发标志
    }
    if (EXTI->PR & (0x00000001 << 6)) // 检测 line6 是否触发
    {
        // 此处编写用户代码 (A6/B6..G6) 引脚触发

        // 此处编写用户代码 (A6/B6..G6) 引脚触发
        EXTI->PR |= (0x00000001 << 6); // 清除 line6 触发标志
    }
    if (EXTI->PR & (0x00000001 << 7)) // 检测 line7 是否触发
    {
        // 此处编写用户代码 (A7/B7..G7) 引脚触发

        // 此处编写用户代码 (A7/B7..G7) 引脚触发
        EXTI->PR |= (0x00000001 << 7); // 清除 line7 触发标志
    }
    if (EXTI->PR & (0x00000001 << 8)) // 检测 line8 是否触发
    {
        camera_vsync_handler();
        // 此处编写用户代码 (A8/B8..G8) 引脚触发

        // 此处编写用户代码 (A8/B8..G8) 引脚触发
        EXTI->PR |= (0x00000001 << 8); // 清除 line8 触发标志
    }
    if (EXTI->PR & (0x00000001 << 9)) // 检测 line9 是否触发
    {
        // 此处编写用户代码 (A9/B9..G9) 引脚触发

        // 此处编写用户代码 (A9/B9..G9) 引脚触发
        EXTI->PR |= (0x00000001 << 9); // 清除 line9 触发标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断 EXTI15-10(A15-10/B15-10..G15-10) 线 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(EXTI15_10_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (0x00000001 << 10)) // 检测 line10 是否触发
    {
        // 此处编写用户代码 (A10/B10..G10) 引脚触发

        // 此处编写用户代码 (A10/B10..G10) 引脚触发
        EXTI->PR |= (0x00000001 << 10); // 清除 line10 触发标志
    }
    if (EXTI->PR & (0x00000001 << 11)) // 检测 line11 是否触发
    {
        // 此处编写用户代码 (A11/B11..G11) 引脚触发

        // 此处编写用户代码 (A11/B11..G11) 引脚触发
        EXTI->PR |= (0x00000001 << 11); // 清除 line11 触发标志
    }
    if (EXTI->PR & (0x00000001 << 12)) // 检测 line12 是否触发
    {
        // 此处编写用户代码 (A12/B12..G12) 引脚触发

        // 此处编写用户代码 (A12/B12..G12) 引脚触发
        EXTI->PR |= (0x00000001 << 12); // 清除 line12 触发标志
    }
    if (EXTI->PR & (0x00000001 << 13)) // 检测 line13 是否触发
    {
        // 此处编写用户代码 (A13/B13..G13) 引脚触发

        // 此处编写用户代码 (A13/B13..G13) 引脚触发
        EXTI->PR |= (0x00000001 << 13); // 清除 line13 触发标志
    }
    if (EXTI->PR & (0x00000001 << 14)) // 检测 line14 是否触发
    {
        // 此处编写用户代码 (A14/B14..G14) 引脚触发

        // 此处编写用户代码 (A14/B14..G14) 引脚触发
        EXTI->PR |= (0x00000001 << 14); // 清除 line14 触发标志
    }
    if (EXTI->PR & (0x00000001 << 15)) // 检测 line15 是否触发
    {
        // 此处编写用户代码 (A15/B15..G15) 引脚触发

        // 此处编写用户代码 (A15/B15..G15) 引脚触发
        EXTI->PR |= (0x00000001 << 15); // 清除 line15 触发标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 1 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH1_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (0 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (0 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 2 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH2_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (1 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (1 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 3 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH3_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel3_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (2 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (2 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 4 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH4_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (3 * 4))) // 判断触发通道
    {
        camera_dma_handler();
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (3 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 5 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH5_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel5_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (4 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (4 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 6 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH6_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel6_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (5 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (5 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 通道 7 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA1_CH7_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA1->ISR & (0x00000001 << (6 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA1->IFCR |= (0x00000001 << (6 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA2 通道 1 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA2_CH1_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA2_Channel1_IRQHandler(void)
{
    if (DMA2->ISR & (0x00000001 << (0 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA2->IFCR |= (0x00000001 << (0 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA2 通道 2 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA2_CH2_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA2_Channel2_IRQHandler(void)
{
    if (DMA2->ISR & (0x00000001 << (1 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA2->IFCR |= (0x00000001 << (1 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA2 通道 3 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA2_CH3_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA2_Channel3_IRQHandler(void)
{
    if (DMA2->ISR & (0x00000001 << (2 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA2->IFCR |= (0x00000001 << (2 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA2 通道 4 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA2_CH4_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA2_Channel4_IRQHandler(void)
{
    if (DMA2->ISR & (0x00000001 << (3 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA2->IFCR |= (0x00000001 << (3 * 4)); // 清空该通道中断标志
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA2 通道 5 的中断服务函数 启动 .s 文件定义 不允许修改函数名称
//              默认优先级 修改优先级使用 interrupt_set_priority(DMA2_CH5_IRQn, 1);
//-------------------------------------------------------------------------------------------------------------------
void DMA2_Channel5_IRQHandler(void)
{
    if (DMA2->ISR & (0x00000001 << (4 * 4))) // 判断触发通道
    {
        // 此处编写用户代码

        // 此处编写用户代码
        DMA2->IFCR |= (0x00000001 << (4 * 4)); // 清空该通道中断标志
    }
}
