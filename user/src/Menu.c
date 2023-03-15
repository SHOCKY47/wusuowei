#include "Menu.h"

typedef struct
{
    int speed;
} app_config;

app_config config = {10};

typedef struct
{
    int menu_id;
    char menu_name[30];
    void (*menu_action)(float32 *param, char name[30]);
    float32 *param;
} menu_item;

menu_item *current_menu_item;

void menu_Departure(void)
{
    system_delay_ms(1500);
    start_flag = 1;
    All_image();
}

void menu_11(void)
{
    ips200_clear();
    key_flag_clear();
    while (key3_flag == 0)
    {
        All_image();
        DrawBoarder(&g_Border); // 原边线
        key_switch();
    }
    return;
}

void menu_12(void)
{
    ips200_clear();
    key_flag_clear();
    while (!key3_flag)
    {
        All_image();
        DrawBoarderInvp(&g_Border); // 逆透视边线
        key_switch();
    }
    return;
}

void menu_13(void)
{
    ips200_clear();
    key_flag_clear();
    while (!key3_flag)
    {
        All_image();
        DrawCenter(&g_Border); // 逆透视中线
        key_switch();
    }
    return;
}

void menu_14(void)
{
    ips200_clear();
    key_flag_clear();
    while (!key3_flag)
    {
        All_image();
        DrawRemoteLine(&g_Border); // 远端边线数组
        key_switch();
    }
    return;
}
void menu_15(void)
{
    ips200_clear();
    key_flag_clear();
    while (!key3_flag)
    {
        All_image();
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
        key_switch();
    }
    return;
}

void menu_tuning(float32 *tuning, char name[30]) // 调参界面菜单
{
    char buf[50];
    char *menu_name = name;
    float32 level[4] = {0.01, 0.1, 1, 10};
    int current_level = 0;
    SEGGER_RTT_printf(0, "\r\n%s: %5.2f", menu_name, *tuning); // 调节挡位开关
    ips200_clear();
    ips200_show_string(1, 0, "Change_level:");
    sprintf(buf, "X%5.2f: ", level[current_level]);
    ips200_show_string(1, 30, buf);
    key_flag_clear();

    while (1)
    {
        if (key_switch())
        {
            if (key1_flag && current_level < 3)
            {
                current_level += 1;
            }
            else if (key2_flag && current_level > 0)
            {
                current_level -= 1;
            }
            else if (key3_flag)
            {
                ips200_clear();
                sprintf(buf, "%s: ", menu_name);
                ips200_show_string(1, 0, buf);
                sprintf(buf, "%5.2f", *tuning);
                ips200_show_string(1, 30, buf);
                key_flag_clear();
                while (1)
                {
                    if (key_switch())
                    {
                        if (key1_flag)
                        {
                            *tuning += level[current_level];
                        }
                        else if (key2_flag)
                        {
                            *tuning -= level[current_level];
                        }
                        else if (key3_flag)
                        {
                            return;
                        }
                        ips200_clear();
                        sprintf(buf, "%s: ", menu_name);
                        ips200_show_string(1, 0, buf);
                        sprintf(buf, "%5.2f", *tuning);
                        ips200_show_string(1, 30, buf);
                        key_flag_clear();
                    }
                }
            }

            ips200_clear();
            ips200_show_string(1, 0, "Change_level:");
            sprintf(buf, "X %5.2f: ", level[current_level]);
            ips200_show_string(1, 30, buf);
            key_flag_clear();
        }
    }
}

menu_item menu[] = {
    {1, "Image Mode", NULL, NULL},                                         // 图像模式
    {11, "Original line", menu_11, NULL},                                  // 显示原边线
    {12, "INV Edge Line", menu_12, NULL},                                  // 显示逆透视边线
    {13, "INV Center Line", menu_13, NULL},                                // 显示逆透视中线
    {14, "Far Side Line", menu_14, NULL},                                  // 显示远端边线
    {15, "All Image Parameters", menu_15, NULL},                           // 显示所有图像参数
    {16, "Back to Main", NULL, NULL},                                      // 回到主菜单
    {2, "Tuning Mode", NULL, NULL},                                        // 调参模式
    {21, "Motor Parameter", NULL, NULL},                                   // 电机参数调整
    {211, "Variable Integral", NULL, NULL},                                // 电机参数调整——变积分
    {2111, "Left Wheel", NULL, NULL},                                      // 电机参数调整——变积分——左轮
    {21111, "Left Wheel MOTOR.L_Max_I", menu_tuning, &MOTOR.L_Max_I},      // 电机参数调整——变积分——左轮Max_I
    {21112, "Left Wheel MOTOR.L_Ci ", menu_tuning, &MOTOR.L_Ci},           // 电机参数调整——变积分——左轮CI
    {21113, "Back to Main", NULL, NULL},                                   // 回到主菜单
    {2112, "Right Wheel", NULL, NULL},                                     // 电机参数调整——变积分——右轮
    {21121, "Right Wheel MOTOR.R_Max_I", menu_tuning, &MOTOR.R_Max_I},     // 电机参数调整——变积分——右轮Max_I
    {21122, "Right Wheel MOTOR.R_Ci", menu_tuning, &MOTOR.R_Ci},           // 电机参数调整——变积分——右轮CI
    {2113, "Back to Main", NULL, NULL},                                    // 回到主菜单
    {21123, "Back to Main", NULL, NULL},                                   // 回到主菜单
    {212, "Variable Ratio", NULL, NULL},                                   // 电机参数调整——变比例
    {2121, "Left Wheel", NULL, NULL},                                      // 电机参数调整——变比例——左轮
    {21211, "Left Wheel MOTOR.L_Bas_KP", menu_tuning, &MOTOR.L_Bas_KP},    // 电机参数调整——变比例——左轮Bas_KP
    {21212, "Left Wheel MOTOR.L_Gain_KP", menu_tuning, &MOTOR.L_Gain_KP},  // 电机参数调整——变比例——左轮Gain_KP
    {21213, "Left Wheel MOTOR.L_Cp", menu_tuning, &MOTOR.L_Cp},            // 电机参数调整——变比例——左轮Cp
    {21214, "Back to Main", NULL, NULL},                                   // 回到主菜单
    {2122, "Right Wheel", NULL, NULL},                                     // 电机参数调整——变比例——右轮
    {21222, "Right Wheel MOTOR.R_Bas_KP", menu_tuning, &MOTOR.R_Bas_KP},   // 电机参数调整——变比例——右轮Bas_KP
    {21222, "Right Wheel MOTOR.R_Gain_KP", menu_tuning, &MOTOR.R_Gain_KP}, // 电机参数调整——变比例——右轮Gain_KP
    {21223, "Right Wheel MOTOR.R_Cp", menu_tuning, &MOTOR.R_Cp},           // 电机参数调整——变比例——右轮Cp
    {21224, "Back to Main", NULL, NULL},                                   // 回到主菜单
    {2123, "Back to Main", NULL, NULL},                                    // 回到主菜单
    {213, "Back to Main", NULL, NULL},                                     // 回到主菜单
    {22, "Duoji Parameter", NULL, NULL},                                   // 舵机参数调整
    {221, "Variable Integral", NULL, NULL},                                // 舵机参数调整——变积分
    {2211, "Serve.Kp_Gain", menu_tuning, &Serve.Kp_Gain},                  // 舵机参数调整——变积分Kp_Gain
    {2212, "Serve.Base", menu_tuning, &Serve.Base},                        // 舵机参数调整——变积分Base
    {2213, "Back to Main", NULL, NULL},                                    // 回到主菜单
    {222, "Variable Ratio", NULL, NULL},                                   // 舵机参数调整——变比例
    {2221, "Serve.Kd_Gain", menu_tuning, &Serve.Kd_Gain},                  // 舵机参数调整——变比例Kd_Gain
    {2222, "Back to Main", NULL, NULL},                                    // 回到主菜单
    {223, "Back to Main", NULL, NULL},                                     // 回到主菜单
    {23, "Back to Main", NULL, NULL},                                      // 回到主菜单
    {3, "Departure Mode", menu_Departure, NULL},                           // 发车模式
    {4, "PESS KEY_4 TO SAVE PARAM", NULL, NULL},                           // 保存所有数据

};

bool have_sub_menu(int menu_id) // 获取当前行位置
{
    for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)
    {
        if (menu[i].menu_id / 10 == menu_id)
        {
            return true;
        }
    }
    return false;
}

int show_sub_menu(int parent_id, int highlight_col) // 当前行显示异色
{
    ips200_clear();
    int item_idx = 0;
    for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)
    {
        if (menu[i].menu_id / 10 == parent_id)
        {
            if (item_idx == highlight_col)
            {
                current_menu_item = &menu[i];
                ips200_set_color(RGB565_RED, RGB565_BLACK);
            }
            else
            {
                ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            }
            ips200_show_string(1, 30 * item_idx, menu[i].menu_name);
            item_idx++;
        }
    }
    return item_idx;
}

void load_config(void)
{
    flash_read_page(127, 0, (uint32 *)&MOTOR, sizeof(MOTOR)); // 从127-0扇区读取MOTOR结构体的数据
    flash_read_page(127, 1, (uint32 *)&Serve, sizeof(Serve)); // 从127-1扇区读取Server结构体的数据
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "Speed: %d\n", config.speed);
}

void save_config(void)
{
    flash_write_page(127, 0, (uint32 *)&MOTOR, sizeof(MOTOR)); // 将MOTOR结构体的数据写入127-0扇区
    flash_write_page(127, 1, (uint32 *)&Serve, sizeof(Serve)); // 将Serve结构体的数据写入127-1扇区
    ips200_clear();
    ips200_show_string(1, 0, "Config saved.Press any key");
    key_flag_clear();

    while (!key_switch())
    {
    }
}

void Menu_Switch(void)
{

    // clock_init(SYSTEM_CLOCK_120M); // 初始化芯片时钟 工作频率为 120MHz
    // debug_init();                  // 初始化默认 Debug UART

    // load_config();

    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY4 输入 默认高电平 上拉输入

    // ips200_init(IPS200_TYPE_PARALLEL8);
    // system_delay_ms(1000);
    // ips200_set_color(RGB565_GREEN, RGB565_BLACK);

    int parent_menu_id = 0; // 目前位置的行号ID
    int highlight_col = 0;  // 高亮行号ID
    int menu_item_count = show_sub_menu(parent_menu_id, highlight_col);
    while (1)
    {
        if (key_switch())
        {
            if (key1_flag && highlight_col > 0)
            {
                highlight_col--;
            }
            else if (key2_flag && highlight_col < menu_item_count - 1)
            {
                highlight_col++;
            }
            else if (key3_flag)
            {
                if (have_sub_menu(current_menu_item->menu_id))
                {
                    highlight_col = 0;
                    parent_menu_id = current_menu_item->menu_id;
                }
                else if (strcmp(current_menu_item->menu_name, "Back to Main") == 0)
                { // 检测到"Back to Main",则返回主菜单界面
                    highlight_col = 0;
                    parent_menu_id = 0;
                }
                else if (current_menu_item->menu_action)
                { // 执行当前行号对应封装函数
                    current_menu_item->menu_action(current_menu_item->param, current_menu_item->menu_name);
                }
            }
            else if (key4_flag)
            {
                save_config();
            }

            menu_item_count = show_sub_menu(parent_menu_id, highlight_col);
            key_flag_clear();
        }
    }
}
