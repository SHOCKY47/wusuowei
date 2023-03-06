#include "PM_Control.h"

int16 normal_speed;
int16 now_speed;
float32 Avg_speed;

// 充电前改变KP参数
uint8 change_kp_flag  = 0;
uint8 leave_kp_flag   = 0;
uint8 choose_flag     = 0;
uint8 charging_switch = 1;

void Control(void)
{
    float32 Img_Error;
    static float32 Img_LastError;

    now_speed    = 40;
    normal_speed = 100;
    Avg_speed    = (Motor_Left.setpoint + Motor_Left.setpoint) / 2;

    if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 1) {
        Img_Error = (g_LineError.m_f32LeftBorderKappa + g_LineError.m_f32RightBorderKappa) / 2.0;
    } else if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 0) {
        Img_Error = g_LineError.m_f32LeftBorderKappa;
    } else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 1) {
        Img_Error = g_LineError.m_f32RightBorderKappa;
    } else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 0) {
        Img_Error = Img_LastError;
    }

    // 需要处理的元素:环岛、P字、S路、坡道、车库
    // 需要固定速度的元素：坡道上、P字出阶段、入库阶段
    // 直道弯道区分 无元素

    if (g_TrackType.m_u8ShortLeftLineStraightFlag == 1 || g_TrackType.m_u8ShortRightLineStraightFlag == 1 /*|| g_TrackType.m_u8CrossFlag != CROSS_NONE || g_TrackType.m_u8RightSideCrossFlag != CROSS_NONE || g_TrackType.m_u8LeftSideCrossFlag != CROSS_NONE*/) {
        Serve.Kp_Gain = 0;
        Serve.Base    = 8;
        Serve.Kd_Gain = 2;
    } else { // 弯道

        Serve.Kp_Gain = 2;
        Serve.Base    = 8;
        Serve.Kd_Gain = 2;
    }

    Duoji_Control(&Steering, &Serve, Img_Error);


    Img_LastError = Img_Error;
}