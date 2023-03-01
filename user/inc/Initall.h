#ifndef __INITALL_H__
#define __INITALL_H__

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

#define LED1         (H2)

#define PIT          (TIM6_PIT)
#define PIT_PRIORITY (TIM6_IRQn)
#define S_MOTOR_PIN  ATOM1_CH1_P33_9

void Initall(void);
void PIT_Init(void);
void Duoji_Init(void);
void Motor_Init(void);
void Encoder_Init(void);
void Key_Init(void);

void DATA_INIT(void);
void Motor_L_Init(void);
void Motor_R_Init(void);
void MOTOR_PID_Init(void);
void Chasu_Init(void);
void Duoji_Data_Init(void);
void Duoji_PID_Init(void);

#endif