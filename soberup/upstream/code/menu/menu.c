#include "menu.h"

#include "menu_nav.h"
#include "menu_params.h"
#include "menu_render.h"
#include "menu_tree.h"
#include "motor.h"
#include "zf_common_headfile.h"
#include "zf_device_ips114.h"

#include <stdio.h>

#define MENU_KEY_DOWN_PIN    (P02_5)
#define MENU_KEY_UP_PIN      (P02_6)
#define MENU_KEY_RIGHT_PIN   (P02_4)
#define MENU_KEY_LEFT_PIN    (P02_7)
#define MENU_KEY_PRESS_PIN   (P00_8)

int8 set_f = 0;
uint8 subject_id = 3U;
uint8 route_direction = ROUTE_DIRECTION_REVERSE;
float basic_speed = 0.0f;
uint16 camera_exposure = 0U;

static MenuContext g_menu_context;
static uint8 map_save_prompt_shown = 0U;
static uint8 map_save_prompt_request = 0U;

static MenuKey menu_scan_key(void)
{
    if (!gpio_get_level(MENU_KEY_UP_PIN)) {
        system_delay_ms(20U);
        while (!gpio_get_level(MENU_KEY_UP_PIN)) {
        }
        system_delay_ms(20U);
        return MENU_KEY_UP;
    }

    if (!gpio_get_level(MENU_KEY_DOWN_PIN)) {
        system_delay_ms(20U);
        while (!gpio_get_level(MENU_KEY_DOWN_PIN)) {
        }
        system_delay_ms(20U);
        return MENU_KEY_DOWN;
    }

    if (!gpio_get_level(MENU_KEY_RIGHT_PIN)) {
        system_delay_ms(20U);
        while (!gpio_get_level(MENU_KEY_RIGHT_PIN)) {
        }
        system_delay_ms(20U);
        return MENU_KEY_RIGHT;
    }

    if (!gpio_get_level(MENU_KEY_LEFT_PIN)) {
        system_delay_ms(20U);
        while (!gpio_get_level(MENU_KEY_LEFT_PIN)) {
        }
        system_delay_ms(20U);
        return MENU_KEY_LEFT;
    }

    if (!gpio_get_level(MENU_KEY_PRESS_PIN)) {
        system_delay_ms(20U);
        while (!gpio_get_level(MENU_KEY_PRESS_PIN)) {
        }
        system_delay_ms(20U);
        return MENU_KEY_PRESS;
    }

    return MENU_KEY_NONE;
}

void menu_init(void)
{
    menu_params_init();
    menu_nav_init(&g_menu_context, menu_tree_root_page());
}

void menu_display(void)
{
    while (!set_f) {
        MenuKey key = menu_scan_key();
        menu_nav_handle_key(&g_menu_context, key);
        menu_nav_tick(&g_menu_context);
        menu_render_current(&g_menu_context);
    }
    ips114_clear();
}

static void menu_map_save_prompt(void)
{
    char buffer[32];

    if (map_save_prompt_shown) {
        return;
    }

    ips114_clear();
    ips114_show_string(0, 16U * 0U, "Save Map?");
    (void)snprintf(buffer, sizeof(buffer), "PWM:%d", max_pwm_output);
    ips114_show_string(0, 16U * 1U, buffer);
    ips114_show_string(0, 16U * 2U, "P:Save  L:No");
    (void)snprintf(buffer, sizeof(buffer), "Pts:%d", Total_Path_Points);
    ips114_show_string(0, 16U * 3U, buffer);
    (void)snprintf(buffer, sizeof(buffer), "STOP:%s",
                   stop_reason_get_text());
    ips114_show_string(0, 16U * 4U, buffer);

    for (;;) {
        MenuKey key = menu_scan_key();

        if (key == MENU_KEY_PRESS) {
            TrajectoryResult result = Save_Trajectory_To_Flash();

            if (result == TRAJECTORY_OK) {
                ips114_show_string(0, 16U * 5U, "SAVED");
            } else {
                ips114_show_string(0, 16U * 5U, "INVALID");
            }
            system_delay_ms(800U);
            break;
        }

        if (key == MENU_KEY_LEFT) {
            ips114_show_string(0, 16U * 5U, "NO SAVE");
            system_delay_ms(800U);
            break;
        }
    }

    map_save_prompt_shown = 1U;
    map_save_prompt_request = 0U;
    ips114_clear();
}

void menu_request_map_save_prompt(void)
{
    if (record_enable &&
        Total_Path_Points > 0 &&
        !map_save_prompt_shown) {
        map_save_prompt_request = 1U;
    }
}

void menu_poll_map_save_prompt(void)
{
    if (!record_enable) {
        map_save_prompt_request = 0U;
        return;
    }

    if (map_save_prompt_request && !map_save_prompt_shown) {
        menu_map_save_prompt();
        map_save_prompt_request = 0U;
    }
}

void menu_reset_save_prompt(void)
{
    map_save_prompt_shown = 0U;
    map_save_prompt_request = 0U;
}
