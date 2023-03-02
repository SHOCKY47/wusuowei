#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

extern int16 normal_speed;
extern int16 now_speed;
extern float32 Avg_speed;

void Control_init(void);
void Control(void);

#endif