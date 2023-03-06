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
    static int lasterror = 0;

    thisError = vPID->setpoint - processValue;

    Duoji->KP = Fabs(thisError * thisError) * Duoji->Kp_Gain + Duoji->Base;
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
    Duoji_Duty = Duoji_Duty >= Duoji_Duty_Maximum ? Duoji_Duty_Maximum : Duoji_Duty;
    Duoji_Duty = Duoji_Duty <= Duoji_Duty_Minimum ? Duoji_Duty_Minimum : Duoji_Duty;
    pwm_set_duty(DUOJI_CHANY, Duoji_Duty);
}