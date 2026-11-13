/**************************************************************
 * 文件名:  initialize.c
 * 创建时间: 2026年3月6日 下午6:13:05
 * 说明:
 **************************************************************/
#include "zf_common_headfile.h"
volatile int start_flag = 0;

void init_mian(void){
    ips114_init();

    while(1)
    {
        if(mt9v03x_init())
            ips114_show_string(0, 80, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500);
    }
    ips114_show_string(0, 16, "init success.");

    Motor_Init();
    NSLEEP_Init();
    Encoder_Init();
    voltage_detect_init();
    LED_init();
    imu660rc_init_main();
    //uart_write_string(UART_2, "hello world");

    // ================= 摄像头预览：确认图像无撕裂 =================
    // 先从 Flash 读取已保存的曝光值并应用到摄像头（曝光存于统一参数页 PARAM_PAGE 的 O_CAMERA_EXP 偏移）
    {
        float img_params[PARAM_WORD_COUNT] = {0};
        if (flash_check(0, PARAM_PAGE) == 1) {
            flash_read_page(0, PARAM_PAGE, (uint32 *)img_params, PARAM_WORD_COUNT);
            if (!isnan(img_params[O_CAMERA_EXP]) && img_params[O_CAMERA_EXP] > 0 && img_params[O_CAMERA_EXP] <= 65535) {
                mt9v03x_set_exposure_time((uint16)img_params[O_CAMERA_EXP]);
            }
        }
    }
//
//    ips114_clear();
//    ips114_show_string(0, 0, "Cam check: press any key");
//    while(1)
//    {
//        if(mt9v03x_finish_flag)
//        {
//            uint8_t thr = otsuThreshold_fast();
//            binarization(thr);
//            ips114_displayimage03x((const uint8 *)mt9v03x_image_bin, 135, 86);
//            mt9v03x_finish_flag = 0;
//        }
//        // 检查所有按键
//        if(!gpio_get_level(KEY_UP) || !gpio_get_level(KEY_DOWN) ||
//           !gpio_get_level(KEY_LEFT) || !gpio_get_level(KEY_RIGHT) ||
//           !gpio_get_level(KEY_PRESS))
//        {
//            system_delay_ms(20);
//            break;
//        }
//    }
    ips114_clear();

    menu_init();
    steer_output_max = basic_speed+200;
    encoder_max_val = 2 * (int)basic_speed;
    menu_display();
    start_flag = 1;
    negative_pressure_start();

    // ================= 初始化2000ms 快速闪烁 LED 灯效 =================
        int total_time = 0;          // 记录已经运行的累计时间
        int current_delay = 150;     // 初始闪烁间隔 150ms

        while(total_time < 3000)
        {
            // 状态 1: LED1亮 LED2亮
            LED1_on();
            LED2_on();
            system_delay_ms(current_delay);
            total_time += current_delay;

            if (total_time >= 2000) break; // 超时保护，提前退出

            // 状态 2: LED1灭 LED2灭
            LED1_off();
            LED2_off();
            system_delay_ms(current_delay);
            total_time += current_delay;

            // 每个循环后缩短延时（每次减小为原来的 75%，实现越来越快）
            current_delay = (current_delay * 3) / 4;

            // 设置最小时间为 10ms，防止闪烁过快
            if(current_delay < 10) {
                current_delay = 10;
            }
        }

        // 退出前确保 LED1 和 LED2 都关闭，保证后续状态干净
    LED1_off();
    LED2_off();
    INS_Init();
    Motor_Reset_State();
    pit_ms_init(CCU60_CH0,2);

}
