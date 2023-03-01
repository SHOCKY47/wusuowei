#ifndef __STEERING_H__
#define __STEERING_H__

#include "zf_common_headfile.h"
#include "headfile.h"

#define DUOJI_CHANY        TIM2_PWM_CH1_A15

#define Duoji_Duty_Maximum 810
#define Duoji_Duty_Midmum  744
#define Duoji_Duty_Minimum 660

extern uint16 Duoji_Duty;

typedef struct // 舵机结构体参数
{

    float KP;
    float KD;

    /*变比例*/
    float Kp_Gain;
    float Base;

    /*变微分*/
    float Kd_Gain;

} Duoji_Para;

typedef struct // 位置型普通PID
{
    float *pPV; //
    float *pSV; //
    float *pMV; //

    float deadband;
    float minimum;
    float maximun;

    float setpoint;

} PID_Serve;

extern Duoji_Para Serve;
extern PID_Serve Steering;

void Duoji_Control(PID_Serve *vPID, Duoji_Para *Duoji, float processValue);
void Duoji_Control_Advance_differential(PID_Serve *vPID, Duoji_Para *Duoji, float processValue);

#endif