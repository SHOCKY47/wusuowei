/*
 * CameraParams.h
 *
 *  Created on: 7 Dec 2021
 *      Author: kare
 */

#ifndef _CAMERAPARAMS_H_
#define _CAMERAPARAMS_H_
#include "zf_common_headfile.h"
#include "imgStruct.h"

/*像素每米*/
extern const float32 PixelperMeter;

/*逆透视矩阵*/
extern const float32 INV_PERS[3][3];

/*逆透视打表变换*/
extern uint8 Inv_x[22560];
extern uint8 Inv_y[22560];

#endif /* CODE_CAMERAPARAMS_H_ */
