#ifndef _QMATH_H_
#define _QMATH_H_

#include "zf_common_headfile.h"
#include "headfile.h"

/*��ƽ��*/
float32 FSqrt(float32 x);

/*ָ������*/
float32 FExp(float32 x);

/*�����У������ݶȷ������*/
uint8 Atan2(float32 y, float32 x);

/*��������*/
void Quicksort(float32 array[], uint8 maxlen, uint8 begin, uint8 end);

/*��ֵ����*/
void Swap(float32 *a, float32 *b);

/*�㽻��*/
void SwapPoint(FLOAT_POINT_INFO *a, FLOAT_POINT_INFO *b);

/*ָ������*/
float32 QPow(float32 base, int8 exp);

/*����ֵ*/
uint16 Kabs(int16 num);

/*����������ֵ����*/
float32 Fabs(float32 num);

/*��Χ����*/
int16 clip(int16 x, int16 low, int16 high);

/*���㷶Χ����*/
float32 fclip(float32 x, float32 low, float32 high);

/*����Ѱ���������ֵ*/
float32 fFindABSMax(float32 *f, int16 len);

/*�����������*/
float32 ThreePointsCurvature(INT_POINT_INFO t_P1, INT_POINT_INFO t_P2, INT_POINT_INFO t_P3);
float32 process_curvity(int16 x1, int16 y1, int16 x2, int16 y2, int16 x3, int16 y3);
float64 Fprocess_curvity(float32 x1, float32 y1, float32 x2, float32 y2, float32 x3, float32 y3);

/*���Իع�*/
void Regression(INT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine);
void FRegression(FLOAT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine);

/*�����������ֵ����*/
void Variance(INT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine);
void FVariance(FLOAT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine);

void LinearRegression(INT_POINT_INFO *Point, int16 startLine1, int16 endLine1, int16 startLine2, int16 endLine2);

#endif /* CODE_QMATH_H_ */
