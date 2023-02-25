#include "Back_Wheel.h"

//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
PID_2 Motor_Left;  // 左轮电机变量
PID_2 Motor_Right; // 右轮电机变量
Motor_Para MOTOR;  // 电机参数结构体变量

Chasu_Para CHASU; // 差速参数结构体变量

float history[4];
float history_Result[4];
float history_DuojiKP[4];

//----------------------------------------------------------参数初始化--------------------------------------------------------------//
//----------------------------------------------------------参数初始化--------------------------------------------------------------//
//----------------------------------------------------------参数初始化--------------------------------------------------------------//
//----------------------------------------------------------参数初始化--------------------------------------------------------------//

void Motor_L_Init(void) // 左轮基本参数初始化
{
    Motor_Left.result      = 0;
    Motor_Left.setpoint    = 100;    // 设定编码器的值
    Motor_Left.maximum     = 12000;  // 输出最大值
    Motor_Left.minimum     = -12000; // 输出最小值
    Motor_Left.deadband    = 0;      // 死区
    Motor_Left.epsilon     = 1000;   // 积分分离(本次偏差是否大于)
    Motor_Left.gama        = 0.6;
    Motor_Left.presetpoint = 100;
}

void Motor_R_Init(void) // 右轮基本参数初始化
{
    Motor_Right.result      = 0;
    Motor_Right.setpoint    = 100;
    Motor_Right.maximum     = 12000;
    Motor_Right.minimum     = -12000;
    Motor_Right.deadband    = 0;    // 死区
    Motor_Right.epsilon     = 1000; // 积分分离(本次偏差是否大于)
    Motor_Right.presetpoint = 100;
}

void MOTOR_PID_Init(void) // 后轮控制参数初始化
{

    /****************************增量式ʽ********************************/

    /****120******/

    //    MOTOR.L_P = 12.5;
    MOTOR.L_I = 1.5;
    MOTOR.L_D = 0;

    //    MOTOR.R_P = 9;
    MOTOR.R_I = 1.5;
    MOTOR.R_D = 0;

    /****120******/

    MOTOR.L_P_120 = 9;
    MOTOR.L_I_120 = 1.5;
    MOTOR.L_D_120 = 0;

    MOTOR.R_P_120 = 9;
    MOTOR.R_I_120 = 1.5;
    MOTOR.R_D_120 = 0;

    /*****变积分参数******/

    /*
     MOTOR.L_P_100 = 15;
     MOTOR.L_Ki_100 = 1.8;
     MOTOR.L_Ci_100 = 0.01;
     */

    /*
     MOTOR.L_P_140 = 12.5;
     MOTOR.L_Ki_140= 1.5;
     MOTOR.L_Ci_140= 0.01;     //Ci越小积分越快
     */

    MOTOR.L_P  = 12.5;
    MOTOR.L_Ki = 1.5;
    MOTOR.L_Ci = 0.01;
    MOTOR.L_Ti = 0;

    MOTOR.R_P  = 12.5;
    MOTOR.R_Ki = 1.5;
    MOTOR.R_Ci = 0.01;
    MOTOR.L_Ti = 0;
}

void Chasu_Init(void) // 差速基本参数初始化
{
    CHASU.K           = 1.8; // 差速系数
    CHASU.result_MAX  = 0.3;
    CHASU.result_MIN  = -0.3;
    CHASU.result      = 0;
    CHASU.Sita        = 0;
    CHASU.Duoji_Error = 0;

    CHASU.L_result_MAX = 20;
    CHASU.L_result_MIN = 10;
    CHASU.R_result_MAX = 20;
    CHASU.R_result_MIN = 10;
}

//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     左轮增量式PID(变积分)
//  @param     PID_1 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID,死区限制，变积分
//--------------------------------------------------------------
void Motor_L_Control_Change_Integral(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{
    float thisError;
    float result;
    float increment;
    float pError, dError, iError;
    float Change_I;

    thisError   = vPID->setpoint - processValue; // 得到偏差值
    Motor->L_Ti = 1.0 / FExp(Motor->L_Ci * Fabs(thisError));
    Change_I    = Motor->L_Ti * Motor->L_Ki;

    result = vPID->result;

    if (Fabs(thisError) > vPID->deadband) // 如果大于死区
    {
        if (vPID->result > vPID->maximum - 1000) { // 如果上次输出结果大于(最大值-1000)，积分值只累计负值
            if (thisError <= 0) {
                iError = thisError;
            }
        } else if (vPID->result < vPID->minimum + 1000) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
            if (thisError >= 0) {
                iError = thisError;
            }
        } else {
            iError = (thisError + vPID->lasterror) / 2.0; // 如果上次输出结果处于正常范围内，正常积分
        }
        pError = thisError - vPID->lasterror;
        dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

        if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差值小，加入积分作用
        {
            increment = Motor->L_P * pError + Change_I * iError + Motor->L_D * dError;
        } else // 如果偏差值大，取消积分作用
        {
            increment = Motor->L_P * pError + Motor->L_D * dError;
        }

    } else // 如果在死区范围内
    {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
        increment = 0.0;
    }
    result = result + increment;

    /*对输出限值，避免超调和积分饱和问题*/
    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }

    vPID->preerror  = vPID->lasterror; // 存放偏差用于下次运算
    vPID->lasterror = thisError;

    vPID->result = result;
    vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
}

//--------------------------------------------------------------
//  @brief     右轮增量式PID(变积分)
//  @param     PID_1 *vPID             速度环PID常用量
//             Motor_Para *Motor       PID参数结构体;
//             processValue            转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID，死区限制，变积分
//--------------------------------------------------------------
void Motor_R_Control_Change_Integral(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{
    float thisError;
    float result;
    float increment;
    float pError, dError, iError;
    float Change_I;

    thisError   = vPID->setpoint - processValue; // 得到偏差值
    Motor->R_Ti = 1.0 / FExp(Motor->R_Ci * Fabs(thisError));
    Change_I    = Motor->R_Ti * Motor->R_Ki;
    result      = vPID->result;

    if (Fabs(thisError) > vPID->deadband) // 如果大于死区
    {
        if (vPID->result > vPID->maximum - 1000) { // 如果上次偏差大于(最大值-1000)，积分值累计负值
            if (thisError <= 0) {
                iError = thisError;
            }
        } else if (vPID->result < vPID->minimum + 1000) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
            if (thisError >= 0) {
                iError = thisError;
            }
        } else {
            iError = (thisError + vPID->lasterror) / 2.0; // 如果上次输出结果处于正常范围内，正常积分
        }
        pError = thisError - vPID->lasterror;
        dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

        if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差值小，加入积分作用
        {
            increment = Motor->R_P * pError + Change_I * iError + Motor->R_D * dError;
        } else // 如果偏差值大，取消积分作用
        {
            increment = Motor->R_P * pError + Motor->R_D * dError;
        }
    } else // 如果在死区范围内
    {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
        increment = 0.0;
    }
    result = result + increment;

    /*对输出限值，避免超调和积分饱和问题*/
    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }

    vPID->preerror  = vPID->lasterror; // 存放偏差用于下次运算
    vPID->lasterror = thisError;

    vPID->result = result;
    vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
}
//--------------------------------------------------------------
//  @brief     左轮增量式PID
//  @param     PID_1 *vPID              速度环PID常用量;
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID，死区限制，梯形积分
//--------------------------------------------------------------
void Motor_L_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    float increment;
    float pError, dError, iError;

    thisError = vPID->setpoint - processValue; // 得到偏差值

    result = vPID->result;

    if (Fabs(thisError) > vPID->deadband) // 如果大于死区
    {
        if (vPID->result > vPID->maximum - 1000) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
            if (thisError <= 0) {
                iError = (thisError + vPID->lasterror) / 2.0;
            }
        } else if (vPID->result < vPID->minimum + 1000) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
            if (thisError >= 0) {
                iError = (thisError + vPID->lasterror) / 2.0;
            }
        } else {
            iError = (thisError + vPID->lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
        }
        pError = thisError - vPID->lasterror;
        dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

        if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
        {
            increment = Motor->L_P * pError + Motor->L_I * iError + Motor->L_D * dError;
        } else // 如果偏差大，取消积分作用
        {
            increment = Motor->L_P * pError + Motor->L_D * dError;
        }

    } else // 如果在死区范围
    {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
        increment = 0.0;
    }
    result = result + increment;

    /*对输出限幅，避免超调和积分饱和问题*/
    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }

    vPID->preerror  = vPID->lasterror; // 存放偏差用于下次运算
    vPID->lasterror = thisError;

    vPID->result = result;
    vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
}

//--------------------------------------------------------------
//  @brief     右轮增量式PID
//  @param     PID_2 *vPID              速度环PID常用量;
//             Motor_Para *Motor        PID参数结构体;
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PI，死区限制，梯形积分
//--------------------------------------------------------------
void Motor_R_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    float increment;
    float pError, dError, iError;

    thisError = vPID->setpoint - processValue; // 得到偏差值

    result = vPID->result;

    if (Fabs(thisError) > vPID->deadband) // 如果大于死区
    {
        if (vPID->result > vPID->maximum - 1000) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
            if (thisError <= 0) {
                iError = (thisError + vPID->lasterror) / 2.0;
            }
        } else if (vPID->result < vPID->minimum + 1000) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
            if (thisError >= 0) {
                iError = (thisError + vPID->lasterror) / 2.0;
            }
        } else {
            iError = (thisError + vPID->lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
        }
        pError = thisError - vPID->lasterror;
        dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

        if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
        {
            increment = Motor->R_P * pError + Motor->R_I * iError + Motor->R_D * dError;
        } else // 如果偏差大，取消积分作用
        {
            increment = Motor->R_P * pError + Motor->R_D * dError;
        }

    } else // 如果在死区范围内
    {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
        increment = 0.0;
    }
    result = result + increment;

    /*对输出限幅，避免超调和积分饱和问题*/
    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }

    vPID->preerror  = vPID->lasterror; // 存放偏差，用于下次运算
    vPID->lasterror = thisError;
    vPID->result    = result;
    vPID->output    = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
}

//--------------------------------------------------------------
//  @brief     左轮位置式PID
//  @param     PID_2 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      位置式PI，死区限制
//--------------------------------------------------------------
void Motor_L_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    thisError = vPID->setpoint - processValue;
    if (Fabs(thisError) > vPID->deadband) {
        vPID->integral += (thisError + vPID->lasterror) / 2;
        result = Motor->L_P * thisError + Motor->L_I * vPID->integral + Motor->L_D * (thisError - vPID->lasterror);

    } else {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
    }
    /*对输出限幅，避免超调和积分饱和问题*/
    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }
    vPID->lasterror = thisError;
    vPID->result    = result;
    vPID->output    = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
}

//--------------------------------------------------------------
//  @brief     右轮位置式PID
//  @param     PID_2 *vPID                   速度环常用量
//             Motor_Para *Motor             PID参数结构体
//             processValue                  转速(编码器值)
//  @return    void        没求得
//  @note      位置式，死区限制
//--------------------------------------------------------------
void Motor_R_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    thisError = vPID->setpoint - processValue;
    if (Fabs(thisError) > vPID->deadband) {
        vPID->integral += (thisError + vPID->lasterror) / 2;
        result = Motor->R_P * thisError + Motor->R_I * vPID->integral + Motor->R_D * (thisError - vPID->lasterror);

    } else {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
    }
    /*对输出限值，避免超调和积分饱和问题*/
    if (result >= vPID->maximum) {
        result = vPID->maximum;
    }
    if (result <= vPID->minimum) {
        result = vPID->minimum;
    }
    vPID->lasterror = thisError;
    vPID->result    = result;
    vPID->output    = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
}
//--------------------------------------------------------------
//  @brief     左轮位置式微分先行PID
//  @param     PID_2 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      位置式PI，死区限制
//--------------------------------------------------------------
void Motor_L_Control_Position_Advance_differential(PID_2 *vPID, Motor_Para *Motor, int16 processValue)

{

    float thisError;
    float c1, c2, c3, temp;

    thisError = vPID->setpoint - processValue;
    vPID->integral += thisError;
    temp             = vPID->gama * Motor->L_D + Motor->L_P;
    c3               = Motor->L_D / temp;
    c2               = (Motor->L_D + Motor->L_P) / temp;
    c1               = vPID->gama * c3;
    vPID->derivative = c1 * vPID->derivative + c2 * processValue + c3 * vPID->lastPv;
    vPID->result     = Motor->L_P * thisError + Motor->L_I * vPID->integral + vPID->derivative;
    vPID->lasterror  = thisError;
    vPID->lastPv     = processValue;
}

//--------------------------------------------------------------差速控制----------------------------------------------------------------//
//--------------------------------------------------------------差速控制----------------------------------------------------------------//
//--------------------------------------------------------------差速控制----------------------------------------------------------------//
//--------------------------------------------------------------差速控制----------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     差速控制
//  @param     Chasu_Para *Diff         差速参数结构体
//  @return    void        没求得
//  @note
//--------------------------------------------------------------
void Differrntial(Chasu_Para *Diff)
{
    Diff->Duoji_Error = Duoji_Duty_final - 740;

    //    Diff->Sita = (Diff->Duoji_Error-1.5)*Pi/2;  //实际测舵机输出角度改变100，转角变化20度,a=(l-1.5)×90°
    if (Fabs(Diff->Duoji_Error) <= 40) // 不差速
    {
        Diff->Sita = 0;
    } else if (Fabs(Diff->Duoji_Error) > 40) {
        Diff->Sita = (Diff->Duoji_Error * 0.2 / 180) * Pi; // 实际测舵机输出角度改变100，转角变化20度
    } else if (Fabs(Diff->Duoji_Error) > 80) {
        Diff->Sita = (Diff->Duoji_Error * 0.2 / 180) * Pi; // 实际测舵机输出角度改变100，转角变化20度
    } else if (Fabs(Diff->Duoji_Error) > 80 && Fabs(Diff->Duoji_Error) < 110) {
        Diff->Sita = (Diff->Duoji_Error * 0.15 / 180) * Pi; // 实际测舵机输出角度改变100，转角变化20度
    } else if (Fabs(Diff->Duoji_Error) <= 20) {
        Diff->Sita = Diff->Duoji_Error * 0.2 / 180 * Pi; // 实际测舵机输出角度改变100，转角变化20度
    } else if (Fabs(Diff->Duoji_Error) >= 20 && Fabs(Diff->Duoji_Error) <= 40) {
        Diff->Sita = Diff->Duoji_Error * 0.2 / 180 * Pi; // 实际测舵机输出角度改变100，转角变化20度
    } else if (Fabs(Diff->Duoji_Error) >= 40) {
        Diff->Sita = Diff->Duoji_Error * 0.2 / 180 * Pi; // 实际测舵机输出角度改变100，转角变化20度
    }

    Diff->result = Fabs(Distance_Row * tanf(Diff->Sita) / (Distance_Col * 2));
    //  2L errorfilter(Diff->result, 0.7); // 滤波
    //  Diff->CS_MAX = DIstance_Row * tanf((MAX->maximum - 1.5) * Pi / 2) / (Distance_Col * 2);

    //    if(Fabs(Diff->result) > Diff->result_MAX )
    //       {
    //        if(Diff->result<0)
    //        {
    //           Diff->result = -Diff->result_MAX;    //限幅
    //        }
    //        else
    //        {
    //           Diff->result =  Diff->result_MAX;
    //        }
    //       }

    if (Diff->result >= Diff->result_MAX) // 限幅
    {
        Diff->result = Diff->result_MAX;
    }
}

//--------------------------------------------------------------
//  @brief     差速输出(改变电机设定速度)
//  @param     Chasu_Para *Diff    差速参数结构体
//             Chasu_V *chasu_L    左轮差速参数
//             Chasu_V *chasu_R    右轮差速参数
//  @return    void        没求得
//  @note
//--------------------------------------------------------------
void Diff_Speed(Chasu_Para *Diff)
{

    //    chasu_L->setpoint = Motor_Left.setpoint * (1 + Diff->K * Diff->result);//双边差速
    //    chasu_R->setpoint = Motor_Right.setpoint * (1 - Diff->K * Diff->result);
    if (Diff->Duoji_Error > 0) // 单边差速
    {
        Motor_Left.setpoint  = Motor_Left.presetpoint * (1 - Diff->K * Diff->result);
        Motor_Right.setpoint = Motor_Right.presetpoint;
        //        chasu_R->setpoint = Motor_Right.presetpoint * (1 + Diff->K * Diff->result);
    } else if (Diff->Duoji_Error <= 0) {
        Motor_Right.setpoint = Motor_Right.presetpoint * (1 - Diff->K * Diff->result);
        Motor_Left.setpoint  = Motor_Left.presetpoint;
        //        chasu_L->setpoint = Motor_Left.presetpoint * (1 + Diff->K * Diff->result);
    }

    if (Motor_Left.setpoint >= CHASU.L_result_MAX) // 限幅
    {
        Motor_Left.setpoint = CHASU.L_result_MAX;
    } else if (Motor_Left.setpoint <= CHASU.L_result_MIN) {
        Motor_Left.setpoint = CHASU.L_result_MIN;
    }

    if (Motor_Right.setpoint >= CHASU.R_result_MAX) // 限幅
    {
        Motor_Right.setpoint = CHASU.R_result_MAX;
    } else if (Motor_Right.setpoint <= CHASU.R_result_MIN) {
        Motor_Right.setpoint = CHASU.R_result_MIN;
    }
}

///**********************被动差速lsd限滑********************/
// float lsd_p=0.1,lsd_pl=0.6,lsd_d=0.8;
// float lsd_p_base=0.15;
// int16 speed_set_l,speed_set_r;
// extern int16 speed_set;     //57大概为1m/s
// extern float speed_p;
// extern float speed_i;
// extern float speed_d;
//
// extern float lsd_show;
// void LSD(int16 encoder_l,int16 encoder_r)
//{
//   static int16 encoder_err;
//   static int16 encoder_det;
//   static int16 encoder_err_last;
//   static float lsd;
//   int turn_err_lsd;
//
//   turn_err_lsd=(int)center-39;
//
//   encoder_err=encoder_l-encoder_r;
////  if((encoder_err>10)||(encoder_err<-10))
////  {
//    encoder_det=encoder_err-encoder_err_last;
//
//    //lsd_p=lsd_p_base+turn_err_lsd*turn_err_lsd/7000;
//
//    //if(lsd_p>0.25) lsd_p=0.25;
//    lsd=encoder_err*lsd_p
//       +encoder_err_last*lsd_pl
//       +encoder_det*lsd_d;
//
//    //在这里加lsd的限幅
//
//    if(lsd>120) lsd=120;
//    if(lsd<-120) lsd=-120;
//    lsd_show=lsd;
//
//    encoder_err_last=encoder_err;
//    speed_set_l=speed_set+(int16)lsd;
//    speed_set_r=speed_set-(int16)lsd;
//
//
//}
//
// int16 Speed_Ctl_l(int16 encoder_l)
//{
//  static int16 speed_err;
//  static int16 speed;
//  static int16 speed_err_last1,speed_err_last2;
//  static int16 speed_bb;
//  speed_err = speed_set_l - encoder_l;    //速度偏差=目标速度-当前速度
//
//  if(speed_err>80) speed_bb=9800;
//  else if(speed_err<-80) speed_bb=-9800;
//  else speed_bb=0;
//  /**********************************/
//  speed+=  speed_p*(speed_err-speed_err_last1)
//                +speed_i* speed_err
//                +speed_d*(speed_err-2*speed_err_last1+speed_err_last2)
//                +speed_bb;
//
//  speed_err_last1=speed_err;
//  speed_err_last2=speed_err_last1;
//
//  if(speed> 9800) speed= 9800;
//  if(speed<-9800) speed=-9800;
//
//  return speed;
//}
//
// int16 Speed_Ctl_r(int16 encoder_r)
//{
//  static int16 speed_err;
//  static int16 speed;
//  static int16 speed_err_last1,speed_err_last2;
//  static int16 speed_bb;
//  speed_err = speed_set_r - encoder_r;    //速度偏差=目标速度-当前速度
//
//  if(speed_err>80) speed_bb=9800;
//  else if(speed_err<-80) speed_bb=-9800;
//  else speed_bb=0;
//  /**********************************/
//  speed+=  speed_p*(speed_err-speed_err_last1)
//                +speed_i* speed_err
//                +speed_d*(speed_err-2*speed_err_last1+speed_err_last2)
//                +speed_bb;
//
//  speed_err_last1=speed_err;
//  speed_err_last2=speed_err_last1;
//
//  if(speed> 9800) speed= 9800;
//  if(speed<-9800) speed=-9800;
//
//  return speed;
//}
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     后轮PWM输出
//  @param     int32 L_outPWM     左电机pwm
//             int32 R_outPWM     右电机pwm
//  @return    void        没求得
//  @note
//--------------------------------------------------------------
void Back_Wheel_Out(int32 L_outPWM, int32 R_outPWM)
{
    if (0 <= L_outPWM) // 电机1 正转
    {

        pwm_set_duty(MOTOR1_PWM, L_outPWM);
        gpio_set_level(MOTOR1_DIR, 0);
    } else // 电机2 反转
    {
        pwm_set_duty(MOTOR1_PWM, -L_outPWM);
        gpio_set_level(MOTOR1_DIR, 1);
    }
    if (0 <= R_outPWM) // 电机2 正转
    {
        pwm_set_duty(MOTOR2_PWM, R_outPWM);
        gpio_set_level(MOTOR2_DIR, 0);
    } else // 电机2 反转
    {
        pwm_set_duty(MOTOR2_PWM, -R_outPWM);
        gpio_set_level(MOTOR2_DIR, 1);
    }
}

//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//
//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//
//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//
//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief       积分分离因子判断函数
//  @param        float error            差值
//                float epsilon        界限
//  @return       beta
//  @note
//--------------------------------------------------------------
uint16 BetaGeneration(float error, float epsilon)
{
    uint16 beta = 0;
    if (Fabs(error) <= epsilon) {
        beta = 1;
    }
    return beta;
}

//--------------------------------------------------------------
//  @brief        均值滤波
//  @param        float inData       输入数据
//                float a            滤波系数
//  @return       outData            输出数据
//  @note         float型
//--------------------------------------------------------------
float data_filtering(float *filter, const float filter_data, const uint8 filter_depth)
{
    float filter_sum = 0;
    uint8 i;
    // 更新数据
    filter[filter_depth] = filter_data;

    for (i = 0; i < filter_depth; i++) {
        filter[i] = filter[i + 1]; // 数据左移，扔掉末尾数据
        filter_sum += filter[i];
    }
    return ((float)filter_sum / (float)filter_depth);
}

//--------------------------------------------------------------
//  @brief        一阶低通滤波
//  @param        float inData       输入数据
//                float a            滤波系数(a* 当前数据 + (1-a)*上次数据)
//  @return       outData            输出数据
//  @note         float型
//--------------------------------------------------------------
float errorfilter(float inData, float a)
{
    float outData;
    static float prevData1R = 0;
    outData                 = (1 - a) * prevData1R + a * inData;
    prevData1R              = inData;
    return outData;
}

//--------------------------------------------------------------
//  @brief        滑动平均滤波(原始数据)
//  @param        float value        输入值
//                uint8 time         取平均个数
//  @return       sum / K            输出数据
//  @note         float型
//--------------------------------------------------------------
float Filter_ave_DuojiData(float value, uint8 time)
{
    int i, j;
    float sum = 0;

    int factor[time];
    int K                   = 0;
    static int filter_index = 0;
    static int buff_init    = 0;

    for (i = 0; i < time; i++) {
        factor[i] = i + 1;
        K += i + 1;
    }
    if (buff_init == 0) {
        history[filter_index] = value;
        filter_index++;

        if (filter_index >= time - 1) {

            buff_init = 1;
        }

    } else {
        history[filter_index] = value;
        filter_index++;
        if (filter_index >= time - 1) {
            filter_index = 0;
        }
        j = filter_index;
        for (i = 0; i <= time - 1; i++) {
            sum += history[j] * factor[i];
            j++;
            if (j == time - 1) {
                j = 0;
            }
        }
    }
    return sum / K;
}
//--------------------------------------------------------------
//  @brief       滑动平均滤波(结果滤波)
//  @param        float value        输入值
//                uint8 time         取平均个数
//  @return       sum / K            输出数据
//  @note         float型
//--------------------------------------------------------------
float Filter_ave_DuojiResult(float value, uint8 time)
{
    static int filter_index = 0;
    static int buff_init    = 0;
    int i, j;
    float sum = 0;

    int factor[time];
    int K = 0;

    for (i = 0; i < time; i++) {
        factor[i] = i + 1;
        K += i + 1;
    }
    if (buff_init == 0) {

        history_Result[filter_index] = value;
        filter_index++;

        if (filter_index >= time - 1) {

            buff_init = 1;
        }

    } else {
        history_Result[filter_index] = value;
        filter_index++;
        if (filter_index >= time - 1) {
            filter_index = 0;
        }
        j = filter_index;
        for (i = 0; i <= time - 1; i++) {
            sum += history_Result[j] * factor[i];
            j++;
            if (j == time - 1) {
                j = 0;
            }
        }
    }
    return sum / K;
}

//--------------------------------------------------------------
//  @brief       滑动平均滤波(对舵机关联的KP值滤波)
//  @param        float value        输入值
//                uint8 time         取平均个数
//  @return       sum / K            输出数据
//  @note         float型
//--------------------------------------------------------------
float Filter_ave_Duoji_KP(float value, uint8 time)
{
    int i, j;
    float sum = 0;

    int factor[time];
    int K                   = 0;
    static int filter_index = 0;
    static int buff_init    = 0;

    for (i = 0; i < time; i++) {
        factor[i] = i + 1;
        K += i + 1;
    }
    if (buff_init == 0) {
        history_DuojiKP[filter_index] = value;
        filter_index++;

        if (filter_index >= time - 1) {

            buff_init = 1;
        }

    } else {
        history_DuojiKP[filter_index] = value;
        filter_index++;
        if (filter_index >= time - 1) {
            filter_index = 0;
        }
        j = filter_index;
        for (i = 0; i <= time - 1; i++) {
            sum += history_DuojiKP[j] * factor[i];
            j++;
            if (j == time - 1) {
                j = 0;
            }
        }
    }
    return sum / K;
}