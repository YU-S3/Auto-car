/**************************************************************
 *  file   : dashedline_makeup.c
 *  desc   : Dashed-line gap filling -- horizontal + vertical.
 *           Marks black pixels between white segments as Make_up
 *           so the 8-neighbor edge crawler traces through dashes.
 *  ported : from Caomang project deal_img.c Dashedline_Makeup()
 *  adapted: Soberup_2026, MT9V03X (188x120), camera height 16cm
 *  coords : row 0=top(far), row 119=bottom(near car)
 *  optimized: direct write to bin (no imgTran), no LandR,
 *             no anomaly filter; column-major vertical pass
 *             tolerated because 188x120 fits close to L1.
 **************************************************************/

#include "dashedline_makeup.h"
#include "image.h"
#include "route_progress.h"

extern uint8 subject_id;
extern uint8 route_direction;

/* expected track width in pixels at each row (horizontal gap limit) */
static const uint8 white_width[MT9V03X_H] = {
     4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    /* row   0- 9 (far)    */
     4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    /* row  10-19          */
     4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    /* row  20-29          */
     4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    /* row  30-39          */
     4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    /* row  40-49          */
     4, 4, 5, 5, 5, 5, 5, 5, 5, 6,    /* row  50-59          */
     6, 6, 6, 6, 6, 7, 7, 7, 7, 7,    /* row  60-69          */
     8, 8, 8, 8, 8, 9, 9, 9, 9, 9,    /* row  70-79          */
    10,10,10,10,10,10,10,10,10,10,    /* row  80-89          */
    11,11,11,11,11,11,11,11,11,11,    /* row  90-99          */
    12,12,12,12,12,12,12,13,13,13,    /* row 100-109         */
    13,13,13,13,13,13,13,13,13,13,    /* row 110-119 (near)  */
};

/* pixel-equivalent of 5cm at each row (vertical gap limit base) */
static const uint8 Length_5cm[MT9V03X_H] = {
    10,10,10,10,10,10,10,10,10,10,    /* row   0- 9 (far)    */
    10,10,10,10,10,10,10,10,10,10,    /* row  10-19          */
    10,10,10,10,10,10,10,10,10,10,    /* row  20-29          */
    10,10,10,10,10,10,10,10,10,10,    /* row  30-39          */
    12,12,12,12,12,12,13,13,13,13,    /* row  40-49          */
    14,14,14,14,14,14,15,15,15,15,    /* row  50-59          */
    16,16,16,16,16,15,15,15,16,16,    /* row  60-69          */
    19,19,19,19,20,20,20,20,21,22,    /* row  70-79          */
    20,20,20,20,21,22,22,22,22,21,    /* row  80-89          */
    22,23,23,23,23,22,23,24,25,25,    /* row  90-99          */
    25,25,25,25,25,26,26,27,27,27,    /* row 100-109         */
    33,33,33,33,33,33,33,33,33,33,    /* row 110-119 (near)  */
};

#pragma optimize 2
uint8 Dashedline_Makeup(void)
{
    int config_index = route_progress_config_fork_index();
    uint8 mode = route_get_fill_mode(
        subject_id, route_direction, config_index);
    if (mode == FILL_NONE) return 0;

    uint8 i;
    int j, k;

    /*==== 1. horizontal gap fill ====*/
    if (mode & FILL_H)
    {
        for (i = DL_Top; i <= DL_Bottom; i++)
        {
            uint8 *row = mt9v03x_image_bin[i];
            int w_range = white_width[i] * 3 / 2;
            j = DL_Left;

            while (j <= DL_Right)
            {
                while (j <= DL_Right && row[j] != 255) j++;
                if (j > DL_Right) break;
                while (j <= DL_Right && row[j] == 255) j++;

                int gap_start = j;
                while (j <= DL_Right && row[j] == 0) j++;
                int gap_end = j - 1;

                if (j <= DL_Right && gap_start <= gap_end
                    && (gap_end - gap_start + 1) <= w_range)
                {
                    for (k = gap_start; k <= gap_end; k++)
                        row[k] = Make_up;
                }
            }
        }
    }

    /*==== 2. vertical gap fill (column-major) ====*/
    if (mode & FILL_V)
    {
        for (j = DL_Left; j <= DL_Right; j++)
        {
            i = DL_Top;

            while (i <= DL_Bottom)
            {
                while (i <= DL_Bottom && mt9v03x_image_bin[i][j] != 255) i++;
                if (i > DL_Bottom) break;
                while (i <= DL_Bottom && mt9v03x_image_bin[i][j] == 255) i++;

                int gap_start = i;
                while (i <= DL_Bottom && mt9v03x_image_bin[i][j] == 0) i++;
                int gap_end = i - 1;

                if (i <= DL_Bottom && gap_start <= gap_end)
                {
                    int h_range = Length_5cm[gap_start - 1] * 3 / 2;
                    if ((gap_end - gap_start + 1) <= h_range)
                    {
                        for (k = gap_start; k <= gap_end; k++)
                            mt9v03x_image_bin[k][j] = Make_up;
                    }
                }
            }
        }
    }

    return 0;
}
#pragma endoptimize
