#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
extern volatile int error;
uint16 time;
void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);
    while(start_flag==0);
    error = 0;
    while (TRUE)
    {
        if(mt9v03x_finish_flag&&stop_flag!=1){
            //route_progress_begin_frame();
            int ther = otsuThreshold_fast();
            binarization(ther);
            Dashedline_Makeup();
            search_line_main();
            state_update();
            caculate_error();
            mt9v03x_finish_flag = 0;
        }
    }
}
#pragma section all restore
