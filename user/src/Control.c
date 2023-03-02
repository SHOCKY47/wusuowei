#include "Control.h"

int16 normal_speed;
int16 now_speed;
float32 Avg_speed;
void Control_init(void)
{
}

void Control(void)
{
    float32 Img_Error;
    static float32 Img_LastError;

    now_speed = 40;

    if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 1) {
        Img_Error = (g_LineError.m_f32LeftBorderKappa + g_LineError.m_f32RightBorderKappa) / 2.0;
    } else if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 0) {
        Img_Error = g_LineError.m_f32LeftBorderKappa;
    } else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 1) {
        Img_Error = g_LineError.m_f32RightBorderKappa;
    } else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 0) {
        Img_Error = Img_LastError;
    }
    // int16 Middle_Speed;
    // Middle_Speed = (High_Speed + Low_Speed) / 2;

    Duoji_Control(&Steering, &Serve, Img_Error);

    Img_LastError = Img_Error;
}