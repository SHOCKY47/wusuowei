#ifndef __PM_CONTROL_H__
#define __PM_CONTROL_H__

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

extern int16 normal_speed;
extern int16 now_speed;
extern float32 Avg_speed;

extern uint8 change_kp_flag;
extern uint8 leave_kp_flag;
extern uint8 choose_flag;
extern uint8 charging_switch;

void Control_init(void);
void Control(void);

#endif