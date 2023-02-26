#include "Steering.h"

uint16 Duoji_Duty;
uint16 Duoji_Duty_final = 740;
uint8 flag              = 1;

Duoji_Para Serve;
PID_Serve Steering;

void Duoji_Data_Init(void)
{
    Steering.setpoint = 0;
    Steering.maximum  = 100;
    Steering.minimum  = -100;
    Steering.deadband = 0;
    Steering.gama     = 0.6;
}

void Duoji_PID_Init(void)
{

    /*****120******/
    Serve.KP = 0;
    Serve.KI = 0;
    Serve.KD = 5;
    //    Serve.KD = 13;
    // 一次曲线
    Serve.K    = 0.1;
    Serve.Bias = 0.6;
    /**************/

    // V函数
    //    Serve.bas_kp = 2.25;
    Serve.bas_kp      = 3.1;
    Serve.b           = 0.8;
    Serve.bas_kd      = 3;
    Serve.straight_kd = 5;
    Serve.KP_V        = 2;
    Serve.KP_fix      = 1.25; // 补偿值
}

//--------------------------------------------------------------
//  @brief     舵机控制函数
//  @param     PID_2 *vPID                       PID结构体
//             processValue                      图像误差
//  @return    void        没有欸
//  @note      位置式
//--------------------------------------------------------------
void Duoji_Control(PID_Serve *vPID, Duoji_Para *Duoji, float processValue)
{

    float thisError;
    float result;
    float KP_kx_b;
    float error_ave;
    float test_KD;

    thisError = vPID->setpoint - processValue;

    error_ave = Filter_ave_DuojiData(Fabs(thisError), 4);
    //    KP_kx_b = Fabs(thisError) * Duoji->K + Duoji->Bias;
    KP_kx_b     = ((Fabs(((FExp(-Fabs(thisError / 8)) - 1)) / ((FExp(-Fabs(thisError / 8)) + 1)))) / 2 + 0.5) * Duoji->Bias;
    Duoji->KP_V = (1 - powf((1 + 1 / Duoji->b), (-error_ave / 6))) * Duoji->bas_kp + Duoji->KP_fix;
    test_KD     = Filter_ave_Duoji_KP(KP_kx_b, 4);

    Duoji->KD = test_KD * Duoji->bas_kd + Duoji->straight_kd;

    if (Fabs(thisError) > vPID->deadband) {
        vPID->integral += (thisError + vPID->lasterror) / 2;
        result = (KP_kx_b + Duoji->KP_V) * thisError + Duoji->KI * vPID->integral + Duoji->KD * (thisError - vPID->lasterror);

    } else {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
    }

    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }
    vPID->lasterror = thisError;
    vPID->result    = result;
    //    vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
    //    ips114_showfloat(187, 0, result, 2, 3);
    Duoji_Duty = Duoji_Duty_Midmum + result;
}

//--------------------------------------------------------------
//  @brief     微分先行舵机控制
//  @param     PID_2 *vPID                       PID结构体
//             processValue                      图像误差
//  @return    void        没有欸
//  @note      微分先行PID
//--------------------------------------------------------------
void Duoji_Control_Advance_differential(PID_Serve *vPID, Duoji_Para *Duoji, float processValue)
{

    float thisError;
    float KP_kx_b;
    float c1, c2, c3, temp;
    float result;

    thisError   = vPID->setpoint - processValue;
    Duoji->KP_V = ((Fabs(((FExp(-Fabs(thisError)) - 1)) / ((FExp(-Fabs(thisError)) + 1)))) / 2 + 0.5) * Duoji->bas_kp;

    //    KP_kx_b = Fabs(thisError)*Duoji->K + Duoji->Bias;

    vPID->integral += thisError;
    temp             = vPID->gama * Duoji->KD + Duoji->KP_V;
    c3               = Duoji->KD / temp;
    c2               = (Duoji->KD + Duoji->KP_V) / temp;
    c1               = vPID->gama * c3;
    vPID->derivative = c1 * vPID->derivative + c2 * processValue + c3 * vPID->lastPv;
    result           = Duoji->KP_V * thisError + Duoji->KI * vPID->integral + vPID->derivative;

    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }
    vPID->lasterror = thisError;
    vPID->lastPv    = processValue;

    vPID->result = result;
    Duoji_Duty   = Duoji_Duty_Midmum - result;
}

void Duoji_Out(uint16 duty)
{
    if (duty > Duoji_Duty_Maximum - 10) {
        duty = Duoji_Duty_Maximum - 10;
    }
    if (duty < Duoji_Duty_Minimum + 10) {
        duty = Duoji_Duty_Minimum + 10;
    }

    pwm_set_duty(DUOJI_CHANY, duty);
}

void Duoji_Test(void)
{
    uint16 i = 800;
    i -= 1;

    Duoji_Out(i);
}
