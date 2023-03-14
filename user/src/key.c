#include "key.h"

uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;
uint8 sw1_status  = 1;
uint8 sw2_status  = 1;

uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status;

char key1_flag, key1_remote_flag;
char key2_flag, key2_remote_flag;
char key3_flag;
char key4_flag;

// uint8 page_max = 8, line_max = 19;
// int8 vision_page = 0, vision_line = 0;

// uint8 key1_flag, key2_flag, key3_flag, key4_flag;

// uint8 start_flag    = 0;
// uint8 wireless_flag = 1;
// uint8 stop_flag     = 1;
// int8 page;

// uint16 i = 744;

// void button(void)
// {
//     if (start_flag) {
//         key3_last_status = key3_status;
//         key4_last_status = key4_status;
//         key3_status      = gpio_get(KEY3_PIN);
//         key4_status      = gpio_get(KEY4_PIN);
//         if (key3_status && !key3_last_status) key3_flag = 1;
//         if (key4_status && !key4_last_status) key4_flag = 1;

//         if (key3_flag || key4_flag) {
//             key3_flag  = 0;
//             key4_flag  = 0;
//             start_flag = 0;
//             stop_flag  = 1;
//         }
//     } else {
//         sw1_status       = gpio_get_level(SW1_PIN);
//         sw2_status       = gpio_get_level(SW2_PIN);
//         key1_last_status = key1_status;
//         key2_last_status = key2_status;
//         key3_last_status = key3_status;
//         key4_last_status = key4_status;
//         // 读取当前按键状态
//         key1_status = gpio_get_level(KEY1_PIN);
//         key2_status = gpio_get_level(KEY2_PIN);
//         key3_status = gpio_get_level(KEY3_PIN);
//         key4_status = gpio_get_level(KEY4_PIN);
//         // 检测到按键按下之后  并放开置位标志位
//         if (key1_status && !key1_last_status) key1_flag = 1;
//         if (key2_status && !key2_last_status) key2_flag = 1;
//         if (key3_status && !key3_last_status) key3_flag = 1;
//         if (key4_status && !key4_last_status) key4_flag = 1;

//         if(sw1_status && key2_flag)   //切换屏幕
//         {
//             key2_flag = 0;//使用按键之后，应该清除标志位
//             if(sw2_status)
//             {
//                 vision_page++;
//                 vision_line = 0;
//                 if(vision_page > page_max)
//                     vision_page = 0;
//             }
//             else
//             {
//                 vision_page--;
//                 vision_line = 0;
//                 if(vision_page < 0)
//                     vision_page = page_max;
//             }
//         }
//                 if(!sw1_status && key2_flag)   //切换行
//         {
//             key2_flag = 0;//使用按键之后，应该清除标志位
//             if(sw2_status)
//                 vision_line++;
//             else
//                 vision_line--;
//             if(vision_line > line_max)
//             {
//                 vision_line = 0;
//             }
//             if(vision_line < 0)
//             {

//                 vision_line = line_max;
//             }

//         }

//     }

//     SWITCH1_status = gpio_get_level(SWITCH1);
//     SWITCH2_status = gpio_get_level(SWITCH2);

//     if (key1_status && !key1_last_status)
//         key1_flag = 1;
//     if (key2_status && !key2_last_status)
//         key2_flag = 1;
//     if (key3_status && !key3_last_status)
//         key3_flag = 1;
//     if (key4_status && !key4_last_status)
//         key4_flag = 1;
//     if (!SWITCH1_status && SWITCH1_last_status)
//         SWITCH1_flag = 1;
//     if (!SWITCH2_status && SWITCH2_last_status)
//         SWITCH2_flag = 1;

//     if (key1_flag) {
//         key1_flag     = 0;
//         Img_Open_falg = !Img_Open_falg;
//         ips200_clear();
//     }
//     if (key2_flag) {
//         key2_flag = 0;
//     }
//     if (key3_flag) {
//         key3_flag = 0;
//     }
//     if (key4_flag) {
//         key4_flag = 0;
//     }
//     if (SWITCH1_flag) {
//         SWITCH1_flag = 1;
//     }
//     if (SWITCH2_flag) {
//         SWITCH2_flag = 1;
//     }
// }
