#ifndef _CROSS_H_
#define _CROSS_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"
// #define CROSS_NONE 0
// #define CROSS_FAR 1
// #define CROSS_NEAR 2

typedef enum {
    CROSS_NONE,
    CROSS_FAR,
    CROSS_NEAR
} CrossStatus;
void DrawRemoteLine(TRACK_BORDER_INFO *p_Border);

void Check_Cross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

void FindRemoteLine(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border);

/*���ݱ��߽Ƕȱ仯������Զ�˽ǵ�*/
void FindRemoteCorner(TRACK_BORDER_INFO *p_Border);

/*����ʮ�ּ��?
 * �ж�����
 * 1. ������?���?
 * 2. �ҵ���������L�͹յ�
 */
void Check_MIDCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*���յ���б��ʮ��
 * �ж�����
 * 1. �����ȫ��?
 * 2. �ұ�L�͹յ�
 *
 * ��������
 * 1. �ҵ�����L�͹յ�
 * 2. �ҵ�����L�͹յ�
 */
void RightThreeCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*���յ���б��ʮ��
 * �ж�����
 * 1. �ұ���?���?
 * 2. ���L�͹յ�
 *
 * ��������
 * 1. �ҵ�����L�͹յ�
 * 2. �ҵ�����L�͹յ�
 */
void LeftThreeCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*���յ���б��ʮ��
 * ����б��ʮ����P��������ͬ��������б��ʮ�ֺ����ұ�����б��ʮ�ּ����־λ��������ʮ�ֺ���Ҫ������Ƿ�Ϊ��P�ֲ�ˢ�������־λ����������������Ԫ��Ҳˢ����־λ��?
 * �ж�����
 * 1. �����ȫ��?
 * 2. �ұ�L�͹յ�
 *
 * ��������
 * 1. �ҵ�����L�͹յ�
 */

void SideCrossFindRemoteLine(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border);

// void RightTwoCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

// /*���յ���б��ʮ��
//  * ����б��ʮ����P��������ͬ��������б��ʮ�ֺ����ұ�����б��ʮ�ּ����־λ��������ʮ�ֺ���Ҫ������Ƿ�Ϊ��P�ֲ�ˢ�������־λ����������������Ԫ��Ҳˢ����־λ��?
//  * �ж�����
//  * 1. �ұ���?���?
//  * 2. ���L�͹յ�
//  *
//  * ��������
//  * 1. �ҵ�����L�͹յ�
//  */
// void LeftTwoCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

#endif /* CODE_CROSS_H_ */
