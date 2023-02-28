#include "Encoder.h"

int16 encoder_data_quaddec_1 = 0;
int16 encoder_data_quaddec_2 = 0;

float V_L = 0;
float V_R = 0;

void Get_Speed(void)
{

    encoder_data_quaddec_1 = encoder_get_count(ENCODER_QUADDEC_1); // 获取编码器计数
    encoder_data_quaddec_2 = encoder_get_count(ENCODER_QUADDEC_2); // 获取编码器计数

    encoder_clear_count(ENCODER_QUADDEC_1); // 清空编码器计数
    encoder_clear_count(ENCODER_QUADDEC_2); // 清空编码器计数

    // V_R = encoder_data_quaddec_1 * 20.4 / (2355.2 * 0.008); // 右轮速度

    // V_L = encoder_data_quaddec_2 * 20.4 / (2355.2 * 0.008); // 左轮速度
}
