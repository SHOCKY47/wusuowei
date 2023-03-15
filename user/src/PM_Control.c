#include "PM_Control.h"

int16 normal_speed;
int16 now_speed;
float32 Avg_speed;

float32 V_Bia; // 差速

float SetLeft;
float SetRight;

// 充电前改变KP参数
uint8 change_kp_flag = 0;
uint8 leave_kp_flag = 0;
uint8 choose_flag = 0;
uint8 charging_switch = 1;

void Angle_Control(void)
{
    float32 Img_Error;
    static float32 Img_LastError;

    if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 1)
    {
        Img_Error = (g_LineError.m_f32LeftBorderKappa + g_LineError.m_f32RightBorderKappa) / 2.0;
    }
    else if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 0)
    {
        Img_Error = g_LineError.m_f32LeftBorderKappa;
    }
    else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 1)
    {
        Img_Error = g_LineError.m_f32RightBorderKappa;
    }
    else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 0)
    {
        Img_Error = Img_LastError;
    }

    // 需要处理的元素:环岛、P字、S路、坡道、车库
    // 需要固定速度的元素：坡道上、P字出阶段、入库阶段
    // 直道弯道区分 无元素

    if (g_TrackType.m_u8ShortLeftLineStraightFlag == 1 || g_TrackType.m_u8ShortRightLineStraightFlag == 1 /*|| g_TrackType.m_u8CrossFlag != CROSS_NONE || g_TrackType.m_u8RightSideCrossFlag != CROSS_NONE || g_TrackType.m_u8LeftSideCrossFlag != CROSS_NONE*/)
    {
        Serve.Kp_Gain = 1;
        Serve.Base = 10;
        Serve.Kd_Gain = 2;
    }
    else
    { // 弯道

        Serve.Kp_Gain = 5;
        Serve.Base = 10;
        Serve.Kd_Gain = 4;
    }

    Duoji_Control(&Steering, &Serve, Img_Error);

    Img_LastError = Img_Error;
}

void Speed_Control(void)
{

    now_speed = 40;
    normal_speed = 100;
    Avg_speed = (Motor_Left.setpoint + Motor_Left.setpoint) / 2;
    SetLeft = Motor_Left.setpoint;
    SetRight = Motor_Right.setpoint;
    V_Bia = Differrntial(&CHASU);

    Get_Speed();

    if (Duoji_Duty > Duoji_Duty_Midmum) // 左转
    {
        SetLeft = SetLeft - V_Bia;
    }

    if (Duoji_Duty < Duoji_Duty_Midmum) // 右转
    {

        SetRight = SetRight - V_Bia;
    }

    Motor_L_Control_Change_Integral(SetLeft, &Motor_Left, &MOTOR, encoder_1);
    Motor_R_Control_Change_Integral(SetRight, &Motor_Right, &MOTOR, encoder_2);

    Back_Wheel_Out(-Motor_Left.result, Motor_Right.result);
}