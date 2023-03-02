#include "Steering.h"

uint16 Duoji_Duty;
Duoji_Para Serve;
PID_Serve Steering;

//--------------------------------------------------------------
//  @brief     位置式PD舵机控制
//  @param     PID_2 *vPID                       PID结构体
//             processValue                      图像误差
//  @return    void        没有欸
//  @note      位置式PD
//--------------------------------------------------------------
void Duoji_Control(PID_Serve *vPID, Duoji_Para *Duoji, float processValue)
{

    float thisError;
    float result;
    static lasterror = 0;

    thisError = vPID->setpoint - processValue;

    Duoji->KP = Fabs(thisError) * thisError * Duoji->Kp_Gain + Duoji->Base;
    Duoji->KD = Duoji->Kd_Gain * Duoji->KP;

    // KP_kx_b     = ((Fabs(((FExp(-Fabs(thisError / 8)) - 1)) / ((FExp(-Fabs(thisError / 8)) + 1)))) / 2 + 0.5) * Duoji->Bias;
    // Duoji->KP_V = (1 - powf((1 + 1 / Duoji->b), (-error_ave / 6))) * Duoji->bas_kp + Duoji->KP_fix;

    if (Fabs(thisError) > vPID->deadband) { // 如果大于死区
        result = Duoji->KP * thisError + Duoji->KD * (thisError - lasterror);

    } else {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
    }

    lasterror  = thisError;
    Duoji_Duty = Duoji_Duty_Midmum + result;
    pwm_set_duty(DUOJI_CHANY, Duoji_Duty);
}

//--------------------------------------------------------------
//  @brief     微分先行舵机控制
//  @param     PID_2 *vPID                       PID结构体
//             processValue                      图像误差
//  @return    void        没有欸
//  @note      微分先行PID
//--------------------------------------------------------------
// void Duoji_Control_Advance_differential(PID_Serve *vPID, Duoji_Para *Duoji, float processValue)
// {

//     float thisError;
//     float KP_kx_b;
//     float c1, c2, c3, temp;
//     float result;

//     thisError   = vPID->setpoint - processValue;
//     Duoji->KP_V = ((Fabs(((FExp(-Fabs(thisError)) - 1)) / ((FExp(-Fabs(thisError)) + 1)))) / 2 + 0.5) * Duoji->bas_kp;

//     //    KP_kx_b = Fabs(thisError)*Duoji->K + Duoji->Bias;

//     vPID->integral += thisError;
//     temp             = vPID->gama * Duoji->KD + Duoji->KP_V;
//     c3               = Duoji->KD / temp;
//     c2               = (Duoji->KD + Duoji->KP_V) / temp;
//     c1               = vPID->gama * c3;
//     vPID->derivative = c1 * vPID->derivative + c2 * processValue + c3 * vPID->lastPv;
//     result           = Duoji->KP_V * thisError + Duoji->KI * vPID->integral + vPID->derivative;

//     if (result >= vPID->maximum) {
//         result = vPID->maximum;
//     }
//     if (result <= vPID->minimum) {
//         result = vPID->minimum;
//     }
//     vPID->lasterror = thisError;
//     vPID->lastPv    = processValue;

//     vPID->result = result;
//     Duoji_Duty   = Duoji_Duty_Midmum - result;
// }
