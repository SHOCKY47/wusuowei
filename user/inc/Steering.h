#ifndef __STEERING_H__
#define __STEERING_H__

#include "zf_common_headfile.h"
#include "headfile.h"

#define Duoji_Duty_Maximum 810
#define Duoji_Duty_Midmum  744
#define Duoji_Duty_Minimum 660

#define DUOJI_CHANY        TIM2_PWM_CH1_A15

extern uint8 flag;
extern uint16 Duoji_Duty;
extern uint16 Duoji_Duty_final;

typedef struct // 舵机结构体参数
{

    float KP;
    float KI;
    float KD;

    /*变比例函数*/
    float K;
    float Bias;

    float KP_V; // V函数
    float bas_kp;
    float b;
    float bas_kd;
    float straight_kd;
    float KP_fix;

} Duoji_Para;

typedef struct // 位置型普通PID
{
    float *pPV; //
    float *pSV; //
    float *pMV; //

    float deadband;
    float maximum;
    float minimum;
    float setpoint;
    float lasterror;
    float result;
    float output;
    float integral;
    float derivative;
    float lastPv;
    float gama;

} PID_Serve;

extern Duoji_Para Serve;
extern PID_Serve Steering;

void Duoji_Data_Init(void);
void Duoji_Control(PID_Serve *vPID, Duoji_Para *Duoji, float processValue);
void Duoji_PID_Init(void);
void Duoji_Out(uint16 duty);
void Duoji_Control_Advance_differential(PID_Serve *vPID, Duoji_Para *Duoji, float processValue);

#endif