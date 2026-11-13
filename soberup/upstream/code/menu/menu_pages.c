#include "menu_pages.h"

#include "menu.h"
#include "route_config.h"
#include "route_progress.h"
#include "run_features.h"
#include "state_control.h"
#include "zf_common_headfile.h"
#include "zf_device_ips114.h"

#include <stdio.h>
#include <string.h>

/* IMG 页面状态与旧实现一致：离开后再次进入仍保留显示模式和调试焦点。 */
static uint8 g_img_show_binary = 1U;
static uint8 g_img_active_param = 0U;
static uint8 g_img_refresh_panel = 0U;
static uint8 g_img_editing = 0U;
static uint8 g_img_digit = 0U;
static uint8 g_img_subpage = 0U;
static uint8 g_img_fwd_active = 0U;
static uint8 g_img_panel_frame_count = 0U;

#define IMG_ROW_EXP   0U
#define IMG_ROW_VIEW  1U
#define IMG_ROW_FRK   2U
#define IMG_ROW_FWD   3U
#define IMG_ROW_DBG   4U
#define IMG_SUB_MAIN  0U
#define IMG_SUB_FWD   1U
#define IMG_SUB_DBG   2U
#define IMG_FWD_NORMAL 0U
#define IMG_FWD_FORK   1U
#define IMG_FWD_BACK   2U
#define IMG_FWD_DETECT 3U

static int *img_active_front_point(void)
{
    if (g_img_fwd_active == IMG_FWD_NORMAL) {
        return &normal_front_point;
    }
    if (g_img_fwd_active == IMG_FWD_FORK) {
        return &fork_front_point;
    }
    if (g_img_fwd_active == IMG_FWD_DETECT) {
        return &angle_detect_lookahead;
    }
    return &fork_exit_lookback_points;
}

static int img_active_front_min(void)
{
    if (g_img_fwd_active == IMG_FWD_BACK ||
        g_img_fwd_active == IMG_FWD_DETECT) {
        return 0;
    }
    return 10;
}

static int img_active_front_max(void)
{
    if (g_img_fwd_active == IMG_FWD_BACK) {
        return 40;
    }
    if (g_img_fwd_active == IMG_FWD_DETECT) {
        return 100;
    }
    return 300;
}

static uint8 img_digit_count(int value)
{
    uint8 digits = 1U;

    if (value < 0) value = -value;
    while (value >= 10) {
        value /= 10;
        digits++;
    }

    return digits;
}

static uint8 img_normalize_digit(int value, uint8 digit)
{
    uint8 digits = img_digit_count(value);

    return (digit < digits) ? digit : (uint8)(digits - 1U);
}

static int img_digit_step(int value, uint8 digit)
{
    uint8 digits = img_digit_count(value);
    int step = 1;

    digit = img_normalize_digit(value, digit);
    while ((uint8)(digit + 1U) < digits) {
        step *= 10;
        digit++;
    }

    return step;
}

static void img_show_highlighted_int(uint16 x, uint16 y, int value, uint8 digit)
{
    char text[8];
    uint8 text_length;
    uint8 i;

    (void)sprintf(text, "%d", value);
    text_length = (uint8)strlen(text);
    if (text_length == 0U) {
        return;
    }

    digit = img_normalize_digit(value, digit);
    for (i = 0U; i < text_length; i++) {
        if (i == digit) {
            ips114_set_color(RGB565_RED, RGB565_WHITE);
        } else {
            ips114_set_color(RGB565_BLACK, RGB565_WHITE);
        }
        ips114_show_char((uint16)(x + i * 8U), y, text[i]);
    }
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
}

static void img_show_padded_text(uint16 x, uint16 y, const char *text, uint8 width)
{
    uint8 i;
    uint8 text_ended = 0U;

    for (i = 0U; i < width; i++) {
        char character = ' ';

        if (text != NULL && !text_ended) {
            character = text[i];
            if (character == '\0') {
                character = ' ';
                text_ended = 1U;
            }
        }
        ips114_show_char((uint16)(x + i * 8U), y, character);
    }
}

static void img_draw_menu_row(uint8 row, const char *label, const char *value)
{
    uint16 y = (uint16)(row * 16U);

    ips114_show_string(136, y,
                       (g_img_active_param == row) ? (g_img_editing ? "*" : ">") : " ");
    ips114_show_string(140, y, label);
    img_show_padded_text(180, y, value, 7U);
}

/*
 * 三叉入口调试字段：
 * EC:x y        x=element_count, y=(element_count == 1)
 * Rxxx y/Lxxx y x=R_point/L_point, y=(point >= 70)
 * Sxxx y A z    x=trend.slope, y=(slope > 80), z=入口总条件是否通过
 * LYx Rn Sxy    x=出口检查层, n=失败原因, xy=左右出口来源
 * TL/TR a/b     a=实际出口步数, b=最低要求步数
 * Oxy           x=有L-only出口, y=有R-only出口
 * Hxyz          x/y/z=左边界/右边界/上边界是否命中
 * YxxXxx Gg Ww  Y/X=出口高度差/水平差, G=几何校验, W=白点过滤
 */
static void menu_page_show_fork_entry_debug_panel(int element_count)
{
    char str[32];
    int ec_ok = (element_count == 1);
    int rp_ok = (R_point >= 70);
    int lp_ok = (L_point >= 70);
    int slope_ok = (trend.slope > 80);
    int all_ok = ec_ok && rp_ok && lp_ok && slope_ok;
    int y_show = (fork_dbg_y_diff > 99) ? 99 : fork_dbg_y_diff;
    int x_show = (fork_dbg_x_diff > 99) ? 99 : fork_dbg_x_diff;
    int slope_show = ((int)trend.slope > 999) ? 999 : (int)trend.slope;

    ips114_show_string(140, 0, "FORK DBG");

    (void)sprintf(str, "EC:%d %d", element_count, ec_ok);
    ips114_show_string(140, 16, str);

    (void)sprintf(str, "R%03d%d L%03d%d", R_point, rp_ok, L_point, lp_ok);
    ips114_show_string(140, 32, str);

    (void)sprintf(str, "S%03d%d A%d", slope_show, slope_ok, all_ok);
    ips114_show_string(140, 48, str);

    (void)sprintf(str, "LY%d R%d S%d%d", fork_dbg_layer, fork_dbg_reason,
                  fork_dbg_src_L, fork_dbg_src_R);
    ips114_show_string(140, 64, str);

    (void)sprintf(str, "TL%03d/%03d TR%03d/%03d", fork_dbg_step_L,
                  fork_dbg_thresh_L, fork_dbg_step_R, fork_dbg_thresh_R);
    ips114_show_string(0, 88, str);

    (void)sprintf(str, "O%d%d H%d%d%d Y%02dX%02d G%d W%d",
                  fork_dbg_has_only_L, fork_dbg_has_only_R,
                  fork_dbg_hit_left, fork_dbg_hit_right, fork_dbg_hit_top,
                  y_show, x_show, fork_dbg_geom_ok, fork_dbg_white_ok);
    ips114_show_string(0, 104, str);
}

static void menu_page_show_image_info_panel(uint8 active_param)
{
    char value[16];
    char status[32];

    (void)active_param;
    (void)sprintf(value, "%d", camera_exposure);
    img_draw_menu_row(IMG_ROW_EXP, "EXP", value);
    img_draw_menu_row(IMG_ROW_VIEW, "VIEW", g_img_show_binary ? "BIN" : "RAW");
    (void)sprintf(value, "%d", fork_num);
    img_draw_menu_row(IMG_ROW_FRK, "FRK", value);
    img_draw_menu_row(IMG_ROW_FWD, "FWD", ">");
    img_draw_menu_row(IMG_ROW_DBG, "DBG", ">");
    if (g_img_editing) {
        if (g_img_active_param == IMG_ROW_EXP) {
            img_show_highlighted_int(180, 0U, (int)camera_exposure, g_img_digit);
        } else if (g_img_active_param == IMG_ROW_FRK) {
            img_show_highlighted_int(180, 32U, fork_num, g_img_digit);
        }
    }
    (void)snprintf(status, sizeof(status), "LP%d RP%d E%d S%d CR%d",
                   L_point, R_point, return_error(), (int)trend.slope,
                   cross_rebuild_result);
    img_show_padded_text(0U, 112U, status, 30U);

    (void)sprintf(value, "AVG %d", img_avg_gray);
    img_show_padded_text(0U, 88U, value, 16U);
}

void menu_page_set_key(MenuContext *context, MenuKey key)
{
    if (key != MENU_KEY_PRESS) {
        return;
    }

    if (run_features_map_enabled()) {
        if (!trajectory_map_valid || Total_Path_Points <= 0) {
            set_f = 0;
            ips114_clear();
            ips114_show_string(48U, 48U, "NO RAM MAP");
            system_delay_ms(1000U);
            menu_nav_request_redraw(context);
            return;
        }
        (void)route_progress_rebuild_map();
        if (fork_correction_enable && !route_progress_valid) {
            set_f = 0;
            ips114_clear();
            ips114_show_string(32U, 48U, "FORK MAP ERR");
            system_delay_ms(1000U);
            menu_nav_request_redraw(context);
            return;
        }
    }

    ips114_clear();
    ips114_show_string(48, 16U * 3U, "Start in 2s");
    system_delay_ms(2000U);

    menu_reset_save_prompt();
    if (record_enable) {
        INS_Clear_Map();
    }
    INS_Init();
    Motor_Reset_State();
    state_reset_run();
    route_progress_reset_run();
    stop_flag = 0;
    IMU_ResetRunDiagnostics();
    set_f = 1;
}

void menu_page_set_render(MenuContext *context)
{
    (void)context;
    ips114_show_string(48, 16 * 2, "P2Go!");
}

void menu_page_img_enter(MenuContext *context)
{
    g_img_subpage = IMG_SUB_MAIN;
    g_img_editing = 0U;
    g_img_panel_frame_count = 0U;
    g_img_refresh_panel = 1U;
    menu_nav_request_redraw(context);
}

void menu_page_img_key(MenuContext *context, MenuKey key)
{
    int direction;

    if (key == MENU_KEY_NONE) {
        return;
    }

    if (g_img_subpage == IMG_SUB_MAIN &&
        !g_img_editing &&
        key == MENU_KEY_LEFT) {
        menu_nav_go_back(context);
        return;
    }

    if (g_img_subpage != IMG_SUB_MAIN) {
        if (g_img_subpage == IMG_SUB_FWD && g_img_editing && (key == MENU_KEY_LEFT || key == MENU_KEY_RIGHT)) {
            int *front_value = img_active_front_point();
            uint8 digits = img_digit_count(*front_value);
            g_img_digit = img_normalize_digit(*front_value, g_img_digit);
            if (key == MENU_KEY_LEFT) g_img_digit = (g_img_digit == 0U) ? (uint8)(digits - 1U) : (uint8)(g_img_digit - 1U);
            else g_img_digit = (uint8)((g_img_digit + 1U) % digits);
        } else if (g_img_subpage == IMG_SUB_FWD && g_img_editing && (key == MENU_KEY_UP || key == MENU_KEY_DOWN)) {
            int *front_value = img_active_front_point();
            direction = (key == MENU_KEY_UP) ? 1 : -1;
            *front_value += direction * img_digit_step(*front_value, g_img_digit);
            if (*front_value < img_active_front_min()) {
                *front_value = img_active_front_min();
            }
            if (*front_value > img_active_front_max()) {
                *front_value = img_active_front_max();
            }
            g_img_digit = img_normalize_digit(*front_value, g_img_digit);
        } else if (g_img_subpage == IMG_SUB_FWD && key == MENU_KEY_PRESS) {
            g_img_editing = (uint8)!g_img_editing;
            g_img_digit = 0U;
        } else if (g_img_subpage == IMG_SUB_FWD && !g_img_editing &&
                   (key == MENU_KEY_UP || key == MENU_KEY_DOWN)) {
            if (key == MENU_KEY_UP) {
                g_img_fwd_active = g_img_fwd_active == IMG_FWD_NORMAL
                    ? IMG_FWD_DETECT
                    : (uint8)(g_img_fwd_active - 1U);
            } else {
                g_img_fwd_active = g_img_fwd_active == IMG_FWD_DETECT
                    ? IMG_FWD_NORMAL
                    : (uint8)(g_img_fwd_active + 1U);
            }
            g_img_digit = 0U;
        }
        else if (key == MENU_KEY_LEFT) { g_img_subpage = IMG_SUB_MAIN; g_img_editing = 0U; menu_nav_request_redraw(context); }
        g_img_refresh_panel = 1U;
        return;
    }
    if (g_img_editing) {
        if (key == MENU_KEY_PRESS) g_img_editing = 0U;
        else if (key == MENU_KEY_LEFT || key == MENU_KEY_RIGHT) {
            int value = (g_img_active_param == IMG_ROW_EXP) ? (int)camera_exposure : fork_num;
            uint8 digits = img_digit_count(value);
            g_img_digit = img_normalize_digit(value, g_img_digit);
            if (key == MENU_KEY_LEFT) g_img_digit = (g_img_digit == 0U) ? (uint8)(digits - 1U) : (uint8)(g_img_digit - 1U);
            else g_img_digit = (uint8)((g_img_digit + 1U) % digits);
        }
        else if (key == MENU_KEY_UP || key == MENU_KEY_DOWN) {
            direction = (key == MENU_KEY_UP) ? 1 : -1;
            if (g_img_active_param == IMG_ROW_EXP) {
                int value = (int)camera_exposure + direction * img_digit_step(camera_exposure, g_img_digit);
                if (value < 10) value = 10; if (value > 65500) value = 65500;
                camera_exposure = (uint16)value; mt9v03x_set_exposure_time(camera_exposure);
                g_img_digit = img_normalize_digit((int)camera_exposure, g_img_digit);
            } else {
                fork_num += direction * img_digit_step(fork_num, g_img_digit);
                if (fork_num < 0) fork_num = 0;
                if (fork_num > FORK_CONFIG_CAPACITY - 1) {
                    fork_num = FORK_CONFIG_CAPACITY - 1;
                }
                g_img_digit = img_normalize_digit(fork_num, g_img_digit);
            }
        }
    } else if (key == MENU_KEY_UP || key == MENU_KEY_DOWN) {
        if (key == MENU_KEY_UP) g_img_active_param = (uint8)((g_img_active_param + 4U) % 5U);
        else g_img_active_param = (uint8)((g_img_active_param + 1U) % 5U);
    } else if (key == MENU_KEY_PRESS) {
        if (g_img_active_param == IMG_ROW_EXP || g_img_active_param == IMG_ROW_FRK) { g_img_editing = 1U; g_img_digit = 0U; }
        else if (g_img_active_param == IMG_ROW_VIEW) g_img_show_binary = (uint8)!g_img_show_binary;
        else if (g_img_active_param == IMG_ROW_FWD) { g_img_subpage = IMG_SUB_FWD; g_img_fwd_active = IMG_FWD_NORMAL; g_img_editing = 0U; menu_nav_request_redraw(context); }
        else { g_img_subpage = IMG_SUB_DBG; menu_nav_request_redraw(context); }
    }
    g_img_refresh_panel = 1U;
}

void menu_page_img_tick(MenuContext *context)
{
    /*
     * 通用渲染器在 tick 之后运行。先消费重绘请求，避免完成一帧图像与
     * 覆盖层绘制后又被通用渲染器清屏。
     */
    if (context != NULL && context->redraw) {
        ips114_clear();
        context->redraw = 0U;
        g_img_refresh_panel = 1U;
    }

    if (mt9v03x_finish_flag) {
        route_progress_begin_frame();
        int threshold = otsuThreshold_fast();
        binarization(threshold);
        Dashedline_Makeup();
        search_line_main();
        state_update();
        caculate_error();

        if (g_img_show_binary) {
            ips114_displayimage03x((const uint8 *)mt9v03x_image_bin, 135, 86);
        } else {
            ips114_show_gray_image(0, 0, (const uint8 *)mt9v03x_image,
                                   MT9V03X_W, MT9V03X_H, 135, 86, 0);
        }

        show_line();
        show_exit_points();
        show_target_point();

        mt9v03x_finish_flag = 0;
        if (g_img_subpage != IMG_SUB_FWD) {
            g_img_panel_frame_count++;
            if (g_img_panel_frame_count >= 4U) {
                g_img_panel_frame_count = 0U;
                g_img_refresh_panel = 1U;
            }
        }
    }
}

void menu_page_img_render(MenuContext *context)
{
    (void)context;

    if (!g_img_refresh_panel) {
        return;
    }

    if (g_img_subpage == IMG_SUB_DBG) {
        menu_page_show_fork_entry_debug_panel(fork_entry_element_count);
    } else if (g_img_subpage == IMG_SUB_FWD) {
        ips114_show_string(140, 0, "FWD");
        ips114_show_string(140, 20,
                           g_img_fwd_active == IMG_FWD_NORMAL
                               ? (g_img_editing ? "*Normal" : ">Normal")
                               : " Normal");
        ips114_show_string(140, 36,
                           g_img_fwd_active == IMG_FWD_FORK
                               ? (g_img_editing ? "*Fork" : ">Fork")
                               : " Fork");
        ips114_show_string(140, 52,
                           g_img_fwd_active == IMG_FWD_BACK
                               ? (g_img_editing ? "*Back" : ">Back")
                               : " Back");
        ips114_show_string(140, 68,
                           g_img_fwd_active == IMG_FWD_DETECT
                               ? (g_img_editing ? "*Detect" : ">Detect")
                               : " Detect");
        if (g_img_editing && g_img_fwd_active == IMG_FWD_NORMAL) {
            img_show_highlighted_int(200, 20U, normal_front_point, g_img_digit);
        } else {
            ips114_show_int(200, 20, normal_front_point, 4);
        }
        if (g_img_editing && g_img_fwd_active == IMG_FWD_FORK) {
            img_show_highlighted_int(200, 36U, fork_front_point, g_img_digit);
        } else {
            ips114_show_int(200, 36, fork_front_point, 4);
        }
        if (g_img_editing && g_img_fwd_active == IMG_FWD_BACK) {
            img_show_highlighted_int(200, 52U,
                                     fork_exit_lookback_points,
                                     g_img_digit);
        } else {
            ips114_show_int(200, 52, fork_exit_lookback_points, 4);
        }
        if (g_img_editing && g_img_fwd_active == IMG_FWD_DETECT) {
            img_show_highlighted_int(200, 68U, angle_detect_lookahead, g_img_digit);
        } else {
            ips114_show_int(200, 68, angle_detect_lookahead, 3);
        }
        img_show_padded_text(140, 84,
                             g_img_editing ? "EDIT" : "PRESS EDIT", 12U);
        ips114_show_string(140, 100, "LEFT BACK");
    } else {
        menu_page_show_image_info_panel(g_img_active_param);
    }
    g_img_refresh_panel = 0U;
}

void menu_page_gyro_calibrate(MenuContext *context)
{
    ips114_clear();
    ips114_show_string(20, 16 * 3, "Keep Still!");
    ips114_show_string(10, 16 * 4, "Wait 0.5s...");
    system_delay_ms(500);

    ips114_show_string(20, 16 * 3, "Calibrating...");
    ips114_show_string(10, 16 * 5, "Keep Still!");

    pit_disable(CCU60_CH1);
    Mahony_CalibrateGyro();
    pit_enable(CCU60_CH1);
    Gyro_SaveOffsets();

    ips114_show_string(30, 16 * 6, "Done!");
    system_delay_ms(800);
    ips114_clear();
    menu_nav_request_redraw(context);
}

void menu_page_encoder_render(MenuContext *context)
{
    char str[30];
    int16 encoder_left;
    int16 encoder_right;

    (void)context;

    /* 读取原始计数但不清零，避免干扰控制逻辑。 */
    encoder_left = encoder_get_count(ENCODER_1);
    encoder_right = encoder_get_count(ENCODER_2);

    ips114_show_string(10, 16 * 2, "Encoder Check");
    (void)sprintf(str, "L: %5d", encoder_left);
    ips114_show_string(10, 16 * 4, str);
    (void)sprintf(str, "R: %5d", encoder_right);
    ips114_show_string(10, 16 * 5, str);
    ips114_show_string(10, 16 * 7, "LEFT to return");
}
