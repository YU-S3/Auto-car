#ifndef CODE_DASHEDLINE_MAKEUP_H_
#define CODE_DASHEDLINE_MAKEUP_H_

#include "zf_device_mt9v03x.h"
#include "route_config.h"

/* pixel markers embedded in binary image (coexist with Black=0 / White=255) */
#define Make_up 128 /* dashed-line gap filler, gray on LCD, visible but distinct */
#define LandR   7   /* black island fully surrounded by white (inside track) */

/* processing region boundaries -- Soberup coords: row 0=top(far), row 119=bottom(near) */
#define DL_Left         2
#define DL_Right        (MT9V03X_W - 3)
#define DL_Top          8
#define DL_Bottom       (MT9V03X_H - 6)
/* inner margin: leave 1px untouched at processing region edges so crawler can stop */
#define DL_In_Left      (DL_Left   + 1)
#define DL_In_Right     (DL_Right  - 1)
#define DL_In_Top       (DL_Top    + 1)
#define DL_In_Bottom    (DL_Bottom - 1)

/* direction bitmask */
#define DL_Goto_U 1
#define DL_Goto_D 2
#define DL_Goto_L 4
#define DL_Goto_R 8

extern uint8 Dashedline_Makeup(void);

#endif /* CODE_DASHEDLINE_MAKEUP_H_ */
