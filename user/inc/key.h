#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"
#include "headfile.h"

#define KEY1_PIN (G0)
#define KEY2_PIN (G1)
#define KEY3_PIN (G2)
#define KEY4_PIN (G3)

#define SW1_PIN  (D14)
#define SW2_PIN  (D15)

extern uint8 key1_status, key2_status, key3_status, key4_status;
extern uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status;
extern char key1_flag, key2_flag, key3_flag, key4_flag;
extern uint8 sw1_status;
extern uint8 sw2_status;

// extern int8 vision_page, vision_line;
// extern uint8 start_flag, wireless_flag;
// extern uint8 stop_flag;
// extern uint8 page_max, line_max;

// void button(void);

#endif
