/*********************************************************************************************************************
* TC264 Opensourec Library
* Copyright (c) 2022 SEEKFREE
********************************************************************************************************************/
#include "zf_common_headfile.h"

#include <stdio.h>

#pragma section all "cpu0_dsram"

static void show_run_diagnostics(uint8_t stopped)
{
    char buffer[24];

    (void)snprintf(buffer, sizeof(buffer), "PWM:%d        ", max_pwm_output);
    ips114_show_string(0U, 0U, buffer);

    if (!stopped)
    {
        return;
    }

    (void)snprintf(buffer, sizeof(buffer), "MC:%lu/%lu        ",
                   (unsigned long)ins_relocalize_success_count,
                   (unsigned long)ins_relocalize_attempt_count);
    ips114_show_string(0U, 16U, buffer);

    if (ins_relocalize_success_count > 0U)
    {
        (void)snprintf(buffer, sizeof(buffer), "D:%+ld        ",
                       (long)ins_relocalize_last_correction_ticks);
    }
    else
    {
        (void)snprintf(buffer, sizeof(buffer), "D:--        ");
    }
    ips114_show_string(0U, 32U, buffer);

    (void)snprintf(buffer, sizeof(buffer), "I:%d/%d O:%d/%d",
                   fork_normal_entry_count,
                   fork_forced_entry_count,
                   fork_normal_exit_count,
                   fork_forced_exit_count);
    ips114_show_string(0U, 48U, buffer);

    (void)snprintf(buffer, sizeof(buffer), "STOP:%s",
                   stop_reason_get_text());
    ips114_show_string(0U, 64U, buffer);
}

int core0_main(void)
{
    clock_init();
    debug_init();

    static uint16 time_used=0;
    static uint32 run_display_last_ms = 0U;
    wireless_uart_init();

    init_mian();
    system_start();
    while (TRUE)
    {
        time_used = system_getval()/100000000.0f;
        uint32 now_ms = system_getval_ms();
        int should_stop = 0;
        if(time_used >= 13 && stop_flag != 1)  should_stop = 1;

        if(should_stop)
        {
            car_stop(STOP_REASON_TIMEOUT);
            start_flag = 0;
            set_f = 0;
            time_used = 0;
        }

        if ((uint32)(now_ms - run_display_last_ms) >= 20U) {
            run_display_last_ms = now_ms;
            show_run_diagnostics((uint8_t)(stop_flag == 1));
        }

        menu_poll_map_save_prompt();

        if (stop_flag != 1) {
            send_speed_to_pc();
            //send_fork_exit_mileage_report_to_pc();
        }
        //system_delay_ms(50);  // 防止主循环全速刷无线串口
    }
}

#pragma section all restore
