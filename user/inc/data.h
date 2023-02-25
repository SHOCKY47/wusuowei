#ifndef CODE_DATA_H_
#define CODE_DATA_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

/*-----------------------��������------------------------------------*/
#define RIGHT 1
#define LEFT  2
#define Pi    3.1415926

extern TRACK_BORDER_INFO g_Border;  /*ȫ��ͼ����ȡ�Ĵ󲿷���Ϣ������������ṹ������*/
extern TRACK_TYPE_INFO g_TrackType; /*��������*/
// extern LINE_ERROR_INFO g_LineError;       /*���ͼ���������ص�ƫ����Ϣ*/

extern uint16 Protect_Frame;

extern uint32 tim1;
extern uint32 tim2;

extern int16 BlackBlock_Monitor;

extern float32 Dist_Monitor;

extern int16 Pnt;

extern int16 GrowCount;
extern int16 GrowCount2;
extern int16 GrowCount3;
extern int16 GrowCount4;

extern uint8 BinaryImg[90][188];

/*-------------------------------��������---------------------------------*/
void data_init(void);

#endif /* CODE_DATA_H_ */
