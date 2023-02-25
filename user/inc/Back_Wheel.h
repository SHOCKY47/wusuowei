#ifndef __BACK_WHEEL_H__
#define __BACK_WHEEL_H__

#include "zf_common_headfile.h"
#include "headfile.h"

#define MOTOR1_DIR   (TIM5_PWM_CH1_A0)
#define MOTOR1_PWM   (TIM5_PWM_CH2_A1)
#define MOTOR2_DIR   (TIM5_PWM_CH3_A2)
#define MOTOR2_PWM   (TIM5_PWM_CH4_A3)

#define Distance_Col 0.200 // 车身的前后轮中心距
#define Distance_Row 0.155 // 两后轮中心距

/*定义PID对象类型*/
typedef struct // 位置型普通PID
{
    float *pPV; // 测量值指针
    float *pSV; // 设定值指针
    float *pMV; // 输出值指针

    float deadband;  /*死区*/
    float maximum;   /*输出值上限*/
    float minimum;   /*输出值下限*/
    float setpoint;  // 设定值
    float lasterror; // 前一拍偏差
    float result;    // 输出值
    float output;    /*输出值0-100%*/
    float integral;  // 积分值

} PID_1;

typedef struct // 增量型普通PID
{
    float *pPV; // 测量值指针
    float *pSV; // 设定值指针
    float *pMV; // 输出值指针

    float deadband;    /*死区*/
    float output;      /*输出值*/
    float maximum;     /*输出值上限*/
    float minimum;     /*输出值下限*/
    float setpoint;    // 设定值
    float presetpoint; // 未加差速设定值
    float lasterror;   // 前一拍偏差
    float preerror;    // 前两拍偏差
    float result;      // 输出值
    float integral;    // 积分值
    float derivative;  // 微分值
    float epsilon;     // 积分分离界限
    float gama;        // 微分先行滤波系数
    float lastPv;      // 上一次的观测值
} PID_2;

typedef struct // 后轮结构体参数
{

    float L_P;
    float L_I;
    float L_D;

    float R_P;
    float R_I;
    float R_D;

    // 左轮参数
    float L_P_120;
    float L_I_120;
    float L_D_120;

    // 右轮参数
    float R_P_120;
    float R_I_120;
    float R_D_120;

    float L_P_100;
    float L_I_100;
    float L_D_100;

    // 右轮参数
    float R_P_100;
    float R_I_100;
    float R_D_100;

    float L_P_70;
    float L_I_70;
    float L_D_70;

    // 右轮参数
    float R_P_70;
    float R_I_70;
    float R_D_70;

    // 变积分参数
    float L_Ti;
    float L_Ki;
    float L_Ci;

    float R_Ti;
    float R_Ki;
    float R_Ci;

} Motor_Para;

typedef struct // 差速结构体参数
{
    float K;           // 补偿系数(K越大差速越提前)
    int16 Duoji_Error; // 舵机设定打角于中值之差
    float Sita;        // 打角角度(转弯半径于车长)
    float result;      // 差速结果
    float result_MAX;  // 差速最大值
    float result_MIN;  // 差速最小值

    float L_result_MAX; // 差速最大值
    float L_result_MIN; // 差速最小值
    float R_result_MAX; // 差速最大值
    float R_result_MIN; // 差速最小值

} Chasu_Para;

extern PID_2 Motor_Left;  // 左轮变量
extern PID_2 Motor_Right; // 右轮变量
extern Motor_Para MOTOR;  // PID参数结构体

extern Chasu_Para CHASU; // 差速参数结构体

extern float history[4]; // 舵机滑动平均滤波数值数组
extern float history_Result[4];
extern float history_DuojiKP[4];

void Back_Wheel_Out(int32 L_outPWM, int32 R_outPWM);
void Motor_L_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
void Motor_R_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
void Motor_L_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
void Motor_R_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
void Motor_L_Control_Position_Advance_differential(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
void Motor_L_Init(void);
void Motor_R_Init(void);
void MOTOR_PID_Init(void);
uint16 BetaGeneration(float error, float epsilon);
float errorfilter(float inData, float a);
float data_filtering(float *filter, const float filter_data, const uint8 filter_depth);
float Filter_ave_DuojiData(float value, uint8 time);
float Filter_ave_DuojiResult(float value, uint8 time);
void Motor_L_Control_Change_Integral(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
void Motor_R_Control_Change_Integral(PID_2 *vPID, Motor_Para *Motor, int16 processValue);
float Filter_ave_Duoji_KP(float value, uint8 time);

void Differrntial(Chasu_Para *Diff);
void Diff_Speed(Chasu_Para *Diff);
void Chasu_Init(void);
void CSV_Left_Init(void);
void CSV_Right_Init(void);

//----------------------------------------------------------我是分割线-------------------------------------------------------------//
//----------------------------------------------------------我是分割线-------------------------------------------------------------//
//----------------------------------------------------------我是分割线-------------------------------------------------------------//
//----------------------------------------------------------我是分割线-------------------------------------------------------------//
// PIDģ模板

////普通位置型PID(梯形积分)
// void PIDRegulation_1(PID_1 *vPID, float processValue)
//{
//
//     float thisError;
//     float result;
//     thisError = vPID->setpoint - processValue;
//     if (fabs(thisError) > vPID->deadband) {
//         vPID->integral += (thisError + vPID->lasterror) / 2;
//         result = vPID->proportiongain * thisError + vPID->integralgain * vPID->integral + vPID->derivativegain * (thisError - vPID->lasterror);
//
//     } else {
//         if ((abs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (abs(processValue - vPID->minimum) < vPID->deadband)) {
//             result = vPID->minimum;
//         }
//     }
//     /*对输出限幅，避免超调和积分饱和问题*/
//     if (result >= vPID->maximum) {
//         result = vPID->maximum;
//     }
//     if (result <= vPID->minimum) {
//         result = vPID->minimum;
//     }
//     vPID->lasterror = thisError;
//     vPID->result = result;
//     vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
// }

// 普通增量型PID(梯形积分)
// void PIDRegulation_2 (PID_2 *vPID, float processValue)
//{
//     float thisError;
//     float result;
//     float increment;
//     float pError, dError, iError;
//
//     thisError = vPID->setpoint - processValue; //得到偏差值
//     result = vPID->result;
//
//     if (fabs(thisError) > vPID->deadband)
//     {
//         pError = thisError - vPID->lasterror;
//         iError = (thisError + vPID->lasterror) / 2.0;
//         dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;
//
//         increment = vPID->proportiongain * pError + vPID->integralgain * iError + vPID->derivativegain * dError;
//     }
//     else
//     {
//         if ((fabs(vPID->setpoint - vPID->minimum) < vPID->deadband)
//                 && (fabs(processValue - vPID->minimum) < vPID->deadband))
//         {
//             result = vPID->minimum;
//         }
//         increment = 0.0;
//     }
//     result = result + increment;
//
//     /*对输出限值，避免超调和积分饱和问题*/
//     if (result >= vPID->maximum)
//     {
//         result = vPID->maximum;
//     }
//     if (result <= vPID->minimum)
//     {
//         result = vPID->minimum;
//     }
//
//     vPID->preerror = vPID->lasterror; //存放偏差用于下次运算
//     vPID->lasterror = thisError;
//     vPID->result = result;
//     vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
// }

#endif