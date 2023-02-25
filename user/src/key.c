#include "key.h"

uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;
uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status;

uint8 motor_flag;
uint8 start_flag = 0;
uint8 stop_flag = 0;
uint8 image_flag = 0;
int8 tt = 1;

uint8 key1_flag, key2_flag, key3_flag, key4_flag;

void Key_Switch (void)
{

    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;

    key1_status = key_get_state(KEY_1);
    key2_status = key_get_state(KEY_2);
    key3_status = key_get_state(KEY_3);
    key4_status = key_get_state(KEY_4);

    if (key1_status && !key1_last_status)
        key1_flag = 1;
    if (key2_status && !key2_last_status)
        key2_flag = 1;
    if (key3_status && !key3_last_status)
        key3_flag = 1;
    if (key4_status && !key4_last_status)
        key4_flag = 1;

    if (key1_flag)
    {
        key1_flag = 0;

        start_flag = 1;

    }
    if (key2_flag)
    {
        key2_flag = 0;

    }
    if (key3_flag)
    {
        key3_flag = 0;

    }
    if (key4_flag)
    {
        key4_flag = 0;
        image_flag = -image_flag;
        
    }
    
}
