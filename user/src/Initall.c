#include "Initall.h"

void Initall(void)
{
    clock_init(SYSTEM_CLOCK_120M); // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                  // 初始化默认 Debug UART

    ips200_init(IPS200_TYPE_PARALLEL8);
    system_delay_ms(1000); // 务必延时

    PIT_Init();
    // timer_init(TIM_2, TIMER_MS);
    mt9v03x_init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式
    sdcardinit();
    Duoji_Init();
    Motor_Init();
    Encoder_Init();
    Key_Init();
    wireless_uart_init();
}

void Duoji_Init(void)
{
    pwm_init(DUOJI_CHANY, 50, Duoji_Duty);
    Duoji_Duty = 744;
}

void Motor_Init(void)
{
    pwm_init(MOTOR1_PWM, 17000, 0);
    gpio_init(MOTOR1_DIR, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);
    gpio_init(MOTOR2_DIR, GPO, 0, GPO_PUSH_PULL);
}

void PIT_Init(void)
{
    pit_ms_init(PIT, 5); // TIM2
    interrupt_set_priority(PIT_PRIORITY, 0);
}

void Encoder_Init(void)
{

    encoder_quad_init(ENCODER_QUADDEC_1, ENCODER_QUADDEC_A_1, ENCODER_QUADDEC_B_1); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUADDEC_2, ENCODER_QUADDEC_A_2, ENCODER_QUADDEC_B_2);
}

void Key_Init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);
}

//----------------------------------------------------------参数初始化--------------------------------------------------------------//
//----------------------------------------------------------参数初始化--------------------------------------------------------------//
//----------------------------------------------------------参数初始化--------------------------------------------------------------//
//----------------------------------------------------------参数初始化--------------------------------------------------------------//

void DATA_INIT(void)
{

    Motor_L_Init();
    Motor_R_Init();
    MOTOR_PID_Init();
    Duoji_Data_Init();
    Duoji_PID_Init();
}

void Motor_L_Init(void) // 左轮基本参数初始化
{
    Motor_Left.result      = 0;
    Motor_Left.setpoint    = 100;   // 设定编码器的值
    Motor_Left.maximum     = 8000;  // 输出最大值
    Motor_Left.minimum     = -8000; // 输出最小值
    Motor_Left.epsilon     = 100;   // 积分分离(本次偏差是否大于)
    Motor_Left.presetpoint = 100;
}

void Motor_R_Init(void) // 右轮基本参数初始化
{
    Motor_Right.result      = 0;
    Motor_Right.setpoint    = 100;
    Motor_Right.maximum     = 8000;
    Motor_Right.minimum     = -8000;
    Motor_Right.epsilon     = 100; // 积分分离(本次偏差是否大于)
    Motor_Right.presetpoint = 100;
}

void MOTOR_PID_Init(void) // 后轮控制参数初始化
{
    /****************************增量式ʽ********************************/

    /****100基本参数*****/
    MOTOR.L_P = 80;
    MOTOR.L_I = 0.15;

    MOTOR.R_P = 80;
    MOTOR.R_I = 0;

    /*****变积分参数******/
    MOTOR.R_Max_I = 0.15;
    MOTOR.L_Ci    = 0.01; // Ci越小积分越快

    MOTOR.R_Max_I = 0.15; // 补偿
    MOTOR.R_Ci    = 0.01;

    /*****变比例参数*****/
    MOTOR.L_Bas_KP  = 50;
    MOTOR.L_Gain_KP = 40;
    MOTOR.L_Cp      = 0.01;

    MOTOR.R_Bas_KP  = 50;
    MOTOR.L_Gain_KP = 40;
    MOTOR.L_Cp      = 0.01;
}

void Duoji_Data_Init(void)
{
    Steering.setpoint = 0;
    Steering.deadband = 0; // 需要确定
    Steering.minimum  = 5;
}

void Duoji_PID_Init(void)
{

    Serve.KP = 0;
    Serve.KD = 0;

    // 变比例参数
    Serve.Kp_Gain = 0.1;
    Serve.Base    = 0.6;

    // 变微分参数
    Serve.Kd_Gain = 2;
}

void Chasu_Init(void) // 差速基本参数初始化
{
    CHASU.K          = 1.8; // 差速系数
    CHASU.result_MAX = 0.3;
    CHASU.result_MIN = -0.3;
    CHASU.result     = 0;
    CHASU.Sita       = 0;

    CHASU.Duoji_Error = 0;
}