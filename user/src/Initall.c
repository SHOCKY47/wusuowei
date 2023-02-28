#include "Initall.h"

void Initall()
{
    clock_init(SYSTEM_CLOCK_120M); // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                  // 初始化默认 Debug UART
    ips200_init(IPS200_TYPE_PARALLEL8);
    system_delay_ms(1000);
    // timer_init(TIM_2, TIMER_MS);
    mt9v03x_init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式
    // sdcardinit();
}