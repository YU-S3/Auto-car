#ifndef CODE_MENU_MENU_H_
#define CODE_MENU_MENU_H_

#include "tuning_presets.h"
#include "route_config.h"
#include "zf_common_typedef.h"

/* 统一参数页：sector 固定为 0，槽32/33用于布局升级。 */
#define PARAM_PAGE             1
#define PARAM_LAYOUT_VERSION   3.0f

#define O_BASIC_SPEED          0
#define O_PID_LEFT             1
#define O_PID_RIGHT            9
#define O_MAX_SPEED           17
#define O_MIN_SPEED           18
#define O_TURNPID             19
#define O_STEER_KP2           26
#define O_TD_FREQ             27
#define O_TD_DAMP             28
#define O_DEAD_ZONE           29
#define O_CAMERA_EXP          30
#define O_FORK_FRONT          31
#define O_FORK_EXIT_LOOKBACK  32
#define O_PARAM_LAYOUT_VERSION 33
#define O_GYRO_X              34
#define O_GYRO_Y              35
#define O_GYRO_Z              36
#define O_GYRO_FLAG           37
#define O_NEGATIVE_PRESS      38
#define O_K_FORWARD           39
#define O_NORMAL_FRONT        40
#define O_ACTIVE_SUBJECT      41
#define O_PRESET_BASE         42
#define O_DECEL_STEP          (O_PRESET_BASE + TUNING_PRESET_STORAGE_WORDS)
#define O_ACCEL_STEP          (O_DECEL_STEP + 1)
#define O_FORK_ENTRY_SHRINK   (O_ACCEL_STEP + 1)
#define O_FORK_EXIT_SHRINK    (O_FORK_ENTRY_SHRINK + 1)
#define O_FORK_ENTRY_MARGIN   (O_FORK_EXIT_SHRINK + 1)
#define O_FORK_EXIT_MARGIN    (O_FORK_ENTRY_MARGIN + 1)
#define O_FORK_CORE_MIN       (O_FORK_EXIT_MARGIN + 1)
#define O_ROUTE_DIRECTION     (O_FORK_CORE_MIN + 1)
#define O_ANGLE_DETECT        (O_ROUTE_DIRECTION + 1)
#define PARAM_WORD_COUNT      (O_ANGLE_DETECT + 1)

void menu_init(void);
void menu_display(void);
void menu_request_map_save_prompt(void);
void menu_poll_map_save_prompt(void);
void menu_reset_save_prompt(void);

extern int8 set_f;
extern uint8 subject_id;
extern uint8 route_direction;
extern float basic_speed;
extern uint16 camera_exposure;

#endif
