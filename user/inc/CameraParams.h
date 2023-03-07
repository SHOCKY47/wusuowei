#ifndef _CAMERAPARAMS_H_
#define _CAMERAPARAMS_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

/*每米的像素点*/
extern const float32 PixelperMeter;

/*逆透视转换矩阵*/
// extern const float32 INV_PERS[3][3];

/*逆透视打表数组*/
extern uint8 Inv_x[22560];
extern uint8 Inv_y[22560];
extern uint8 Inv_x_imge[22560];
extern uint8 Inv_y_imge[22560];
// extern uint8 Inv_x2[22560];
// extern uint8 Inv_y2[22560];
// extern uint8 Inv_x_imge2[22560];
// extern uint8 Inv_y_imge2[22560];

#endif /* CODE_CAMERAPARAMS_H_ */
