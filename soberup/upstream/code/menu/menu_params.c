#include "menu.h"
#include "menu_params.h"
#include "route_progress.h"

#include "zf_common_headfile.h"

#include <math.h>
#include <string.h>

extern PosPID pid_left;
extern PosPID pid_right;
extern SteeringPID turnPID;
extern float steer_Kp2;
extern float td_freq;
extern float td_damp;
extern float dead_zone;
extern int min_target_speed;
extern int max_target_speed;

typedef char menu_pospid_size_must_be_32[(sizeof(PosPID) == 32U) ? 1 : -1];
typedef char menu_steering_size_must_be_28[(sizeof(SteeringPID) == 28U) ? 1 : -1];
typedef char menu_param_page_capacity_check[
    (PARAM_WORD_COUNT <= EEPROM_PAGE_LENGTH) ? 1 : -1];

static float params[PARAM_WORD_COUNT] = {0.0f};

static uint8 menu_params_stepped_int_valid(
    float value,
    int minimum,
    int maximum,
    int step)
{
    int integer_value;

    if (isnan(value) || value < (float)minimum || value > (float)maximum)
    {
        return 0U;
    }
    integer_value = (int)value;
    return (uint8)(
        value == (float)integer_value &&
        step > 0 && integer_value % step == 0);
}

static void menu_params_write_full_page(void)
{
    params[O_FORK_EXIT_LOOKBACK] = (float)fork_exit_lookback_points;
    params[O_PARAM_LAYOUT_VERSION] = PARAM_LAYOUT_VERSION;
    tuning_presets_store(&params[O_PRESET_BASE]);
    params[O_DECEL_STEP] = (float)DECEL_STEP;
    params[O_ACCEL_STEP] = (float)ACCEL_STEP;
    params[O_FORK_ENTRY_SHRINK] = (float)fork_entry_shrink_percent;
    params[O_FORK_EXIT_SHRINK] = (float)fork_exit_shrink_percent;
    params[O_FORK_ENTRY_MARGIN] = (float)fork_entry_margin_ticks;
    params[O_FORK_EXIT_MARGIN] = (float)fork_exit_margin_ticks;
    params[O_FORK_CORE_MIN] = (float)fork_hard_core_min_ticks;
    params[O_ROUTE_DIRECTION] = (float)route_direction;
    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}
static void menu_params_read_or_initialize(void)
{
    uint8 need_init = 0;

    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);

        if (isnan(params[0]) || isnan(params[1]))
        {
            need_init = 1;
        }
    }
    else
    {
        need_init = 1;
    }

    if (need_init)
    {
        memset(params, 0, sizeof(params));

        params[O_BASIC_SPEED] = basic_speed;
        memcpy(&params[O_PID_LEFT], &pid_left, sizeof(PosPID));
        memcpy(&params[O_PID_RIGHT], &pid_right, sizeof(PosPID));
        params[O_MAX_SPEED] = 1200.0f;
        params[O_MIN_SPEED] = 400.0f;
        memcpy(&params[O_TURNPID], &turnPID, sizeof(SteeringPID));
        params[O_STEER_KP2] = 0.00f;
        params[O_TD_FREQ] = 15.0f;
        params[O_TD_DAMP] = 1.0f;
        params[O_DEAD_ZONE] = 5.0f;
        params[O_CAMERA_EXP] = (float)MT9V03X_EXP_TIME_DEF;
        params[O_FORK_FRONT] = 125.0f;
        params[O_FORK_EXIT_LOOKBACK] = 30.0f;
        params[O_PARAM_LAYOUT_VERSION] = PARAM_LAYOUT_VERSION;
        params[O_NEGATIVE_PRESS] = 50.0f;
        params[O_GYRO_FLAG] = 0.0f;
        params[O_K_FORWARD] = 0.0f;
        params[O_NORMAL_FRONT] = 105.0f;
        params[O_ANGLE_DETECT] = 20.0f;
        params[O_ACTIVE_SUBJECT] = 3.0f;
        params[O_ROUTE_DIRECTION] = (float)ROUTE_DIRECTION_REVERSE;
        params[O_DECEL_STEP] = (float)TRAJECTORY_DECEL_DEFAULT;
        params[O_ACCEL_STEP] = (float)TRAJECTORY_ACCEL_DEFAULT;
        params[O_FORK_ENTRY_SHRINK] = 40.0f;
        params[O_FORK_EXIT_SHRINK] = 20.0f;
        params[O_FORK_ENTRY_MARGIN] = 5000.0f;
        params[O_FORK_EXIT_MARGIN] = 3000.0f;
        params[O_FORK_CORE_MIN] = 1000.0f;

        menu_params_write_full_page();
    }
}

void menu_params_init(void)
{
    uint8 need_save = 0;
    float stored_layout_version;

    memset(params, 0, sizeof(params));
    menu_params_read_or_initialize();
    stored_layout_version = params[O_PARAM_LAYOUT_VERSION];

    if (stored_layout_version != 2.0f &&
        stored_layout_version != PARAM_LAYOUT_VERSION)
    {
        /* 版本1的预设每组只有11个word，不能按当前12-word步长读取。 */
        fork_exit_lookback_points = 30;
        params[O_FORK_EXIT_LOOKBACK] =
            (float)fork_exit_lookback_points;
        memset(&params[O_PRESET_BASE], 0,
               TUNING_PRESET_STORAGE_WORDS * sizeof(params[0]));
        need_save = 1;
    }
    else if (!isnan(params[O_FORK_EXIT_LOOKBACK]) &&
             params[O_FORK_EXIT_LOOKBACK] >= 0.0f &&
             params[O_FORK_EXIT_LOOKBACK] <= 40.0f)
    {
        fork_exit_lookback_points =
            (int)params[O_FORK_EXIT_LOOKBACK];
    }
    else
    {
        fork_exit_lookback_points = 30;
        params[O_FORK_EXIT_LOOKBACK] =
            (float)fork_exit_lookback_points;
        need_save = 1;
    }

    if (stored_layout_version != PARAM_LAYOUT_VERSION)
    {
        params[O_FORK_ENTRY_SHRINK] = 40.0f;
        params[O_FORK_EXIT_SHRINK] = 20.0f;
        params[O_FORK_ENTRY_MARGIN] = 5000.0f;
        params[O_FORK_EXIT_MARGIN] = 3000.0f;
        params[O_FORK_CORE_MIN] = 1000.0f;
        params[O_PARAM_LAYOUT_VERSION] = PARAM_LAYOUT_VERSION;
        need_save = 1;
    }

    basic_speed = params[O_BASIC_SPEED];
    memcpy(&pid_left, &params[O_PID_LEFT], sizeof(PosPID));
    memcpy(&pid_right, &params[O_PID_RIGHT], sizeof(PosPID));

    if (params[O_MAX_SPEED] != 0 || params[O_MIN_SPEED] != 0)
    {
        max_target_speed = (int)params[O_MAX_SPEED];
        min_target_speed = (int)params[O_MIN_SPEED];
    }
    else
    {
        max_target_speed = 1200;
        min_target_speed = 400;
        params[O_MAX_SPEED] = (float)max_target_speed;
        params[O_MIN_SPEED] = (float)min_target_speed;
        need_save = 1;
    }

    memcpy(&turnPID, &params[O_TURNPID], sizeof(SteeringPID));

    if (params[O_STEER_KP2] != 0.0f || params[O_TD_FREQ] != 0.0f)
    {
        steer_Kp2 = params[O_STEER_KP2];
        td_freq = (params[O_TD_FREQ] >= 1.0f && params[O_TD_FREQ] <= 100.0f)
                    ? params[O_TD_FREQ] : 15.0f;
        td_damp = (params[O_TD_DAMP] >= 0.5f && params[O_TD_DAMP] <= 2.0f)
                    ? params[O_TD_DAMP] : 1.0f;
        dead_zone = (params[O_DEAD_ZONE] >= 0.0f && params[O_DEAD_ZONE] <= 50.0f)
                    ? params[O_DEAD_ZONE] : 5.0f;
    }
    else
    {
        steer_Kp2 = 0.00f;
        td_freq = 15.0f;
        td_damp = 1.0f;
        dead_zone = 5.0f;
        params[O_STEER_KP2] = steer_Kp2;
        params[O_TD_FREQ] = td_freq;
        params[O_TD_DAMP] = td_damp;
        params[O_DEAD_ZONE] = dead_zone;
        need_save = 1;
    }

    if (params[O_K_FORWARD] >= 0.0f && params[O_K_FORWARD] <= 10.0f)
    {
        K_forward = params[O_K_FORWARD];
    }
    else
    {
        K_forward = 0.0f;
        params[O_K_FORWARD] = K_forward;
        need_save = 1;
    }

    if (params[O_NEGATIVE_PRESS] >= 0.0f && params[O_NEGATIVE_PRESS] <= 100.0f)
    {
        negative_pressure_percent = params[O_NEGATIVE_PRESS];
    }
    else
    {
        negative_pressure_percent = 50.0f;
        params[O_NEGATIVE_PRESS] = negative_pressure_percent;
        need_save = 1;
    }

    if (!isnan(params[O_ACTIVE_SUBJECT])
        && (params[O_ACTIVE_SUBJECT] == 2.0f || params[O_ACTIVE_SUBJECT] == 3.0f))
    {
        subject_id = (uint8)params[O_ACTIVE_SUBJECT];
    }
    else
    {
        subject_id = 3U;
        params[O_ACTIVE_SUBJECT] = (float)subject_id;
        need_save = 1;
    }

    if (!isnan(params[O_ROUTE_DIRECTION]) &&
        (params[O_ROUTE_DIRECTION] == (float)ROUTE_DIRECTION_FORWARD ||
         params[O_ROUTE_DIRECTION] == (float)ROUTE_DIRECTION_REVERSE))
    {
        route_direction = (uint8)params[O_ROUTE_DIRECTION];
    }
    else
    {
        route_direction = ROUTE_DIRECTION_REVERSE;
        params[O_ROUTE_DIRECTION] = (float)route_direction;
        need_save = 1;
    }

    if (!isnan(params[O_CAMERA_EXP])
        && params[O_CAMERA_EXP] > 0
        && params[O_CAMERA_EXP] <= 65535)
    {
        camera_exposure = (uint16)params[O_CAMERA_EXP];
    }
    else
    {
        camera_exposure = MT9V03X_EXP_TIME_DEF;
        params[O_CAMERA_EXP] = (float)camera_exposure;
        need_save = 1;
    }
    mt9v03x_set_exposure_time(camera_exposure);

    if (!isnan(params[O_FORK_FRONT])
        && params[O_FORK_FRONT] >= 10.0f
        && params[O_FORK_FRONT] <= 300.0f)
    {
        fork_front_point = (int)params[O_FORK_FRONT];
    }
    else
    {
        fork_front_point = 125;
        params[O_FORK_FRONT] = (float)fork_front_point;
        need_save = 1;
    }

    if (!isnan(params[O_NORMAL_FRONT])
        && params[O_NORMAL_FRONT] >= 10.0f
        && params[O_NORMAL_FRONT] <= 300.0f)
    {
        normal_front_point = (int)params[O_NORMAL_FRONT];
    }
    else
    {
        normal_front_point = 105;
        params[O_NORMAL_FRONT] = (float)normal_front_point;
        need_save = 1;
    }

    if (!isnan(params[O_ANGLE_DETECT])
        && params[O_ANGLE_DETECT] >= 0.0f
        && params[O_ANGLE_DETECT] <= 100.0f)
    {
        angle_detect_lookahead = (int)params[O_ANGLE_DETECT];
    }
    else
    {
        angle_detect_lookahead = 20;
        params[O_ANGLE_DETECT] = (float)angle_detect_lookahead;
        need_save = 1;
    }

    if (tuning_presets_load(&params[O_PRESET_BASE]))
    {
        need_save = 1;
    }

    if (!isnan(params[O_DECEL_STEP]) &&
        params[O_DECEL_STEP] >= (float)TRAJECTORY_STEP_MIN &&
        params[O_DECEL_STEP] <= (float)TRAJECTORY_STEP_MAX)
    {
        DECEL_STEP = (int)params[O_DECEL_STEP];
    }
    else
    {
        DECEL_STEP = TRAJECTORY_DECEL_DEFAULT;
        params[O_DECEL_STEP] = (float)DECEL_STEP;
        need_save = 1;
    }

    if (!isnan(params[O_ACCEL_STEP]) &&
        params[O_ACCEL_STEP] >= (float)TRAJECTORY_STEP_MIN &&
        params[O_ACCEL_STEP] <= (float)TRAJECTORY_STEP_MAX)
    {
        ACCEL_STEP = (int)params[O_ACCEL_STEP];
    }
    else
    {
        ACCEL_STEP = TRAJECTORY_ACCEL_DEFAULT;
        params[O_ACCEL_STEP] = (float)ACCEL_STEP;
        need_save = 1;
    }

    if (menu_params_stepped_int_valid(
            params[O_FORK_ENTRY_SHRINK], 0, 45, 5))
    {
        fork_entry_shrink_percent = (int)params[O_FORK_ENTRY_SHRINK];
    }
    else
    {
        fork_entry_shrink_percent = 40;
        params[O_FORK_ENTRY_SHRINK] = 40.0f;
        need_save = 1;
    }

    if (menu_params_stepped_int_valid(
            params[O_FORK_EXIT_SHRINK], 0, 45, 5))
    {
        fork_exit_shrink_percent = (int)params[O_FORK_EXIT_SHRINK];
    }
    else
    {
        fork_exit_shrink_percent = 20;
        params[O_FORK_EXIT_SHRINK] = 20.0f;
        need_save = 1;
    }

    if (menu_params_stepped_int_valid(
            params[O_FORK_ENTRY_MARGIN], 0, 10000, 500))
    {
        fork_entry_margin_ticks = (int)params[O_FORK_ENTRY_MARGIN];
    }
    else
    {
        fork_entry_margin_ticks = 5000;
        params[O_FORK_ENTRY_MARGIN] = 5000.0f;
        need_save = 1;
    }

    if (menu_params_stepped_int_valid(
            params[O_FORK_EXIT_MARGIN], 0, 10000, 500))
    {
        fork_exit_margin_ticks = (int)params[O_FORK_EXIT_MARGIN];
    }
    else
    {
        fork_exit_margin_ticks = 3000;
        params[O_FORK_EXIT_MARGIN] = 3000.0f;
        need_save = 1;
    }

    if (menu_params_stepped_int_valid(
            params[O_FORK_CORE_MIN], 0, 10000, 500))
    {
        fork_hard_core_min_ticks = (int)params[O_FORK_CORE_MIN];
    }
    else
    {
        fork_hard_core_min_ticks = 1000;
        params[O_FORK_CORE_MIN] = 1000.0f;
        need_save = 1;
    }

    if (need_save)
    {
        menu_params_write_full_page();
    }
    front_point = normal_front_point;
}

void menu_params_save_steering(void)
{
    memset(params, 0, sizeof(params));
    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);
    }

    memcpy(&params[O_TURNPID], &turnPID, sizeof(SteeringPID));
    params[O_STEER_KP2] = steer_Kp2;
    params[O_TD_FREQ] = td_freq;
    params[O_TD_DAMP] = td_damp;
    params[O_DEAD_ZONE] = dead_zone;

    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}

void menu_params_save_speed(void)
{
    memset(params, 0, sizeof(params));
    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);
    }

    params[O_BASIC_SPEED] = basic_speed;
    memcpy(&params[O_PID_LEFT], &pid_left, sizeof(PosPID));
    memcpy(&params[O_PID_RIGHT], &pid_right, sizeof(PosPID));
    params[O_K_FORWARD] = K_forward;
    params[O_NEGATIVE_PRESS] = negative_pressure_percent;

    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}

void menu_params_save_image(void)
{
    memset(params, 0, sizeof(params));
    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);
    }

    params[O_CAMERA_EXP] = (float)camera_exposure;
    params[O_FORK_FRONT] = (float)fork_front_point;
    params[O_NORMAL_FRONT] = (float)normal_front_point;
    params[O_ANGLE_DETECT] = (float)angle_detect_lookahead;
    params[O_FORK_EXIT_LOOKBACK] = (float)fork_exit_lookback_points;
    params[O_PARAM_LAYOUT_VERSION] = PARAM_LAYOUT_VERSION;

    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}

void menu_params_save_subject(void)
{
    memset(params, 0, sizeof(params));
    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);
    }

    params[O_ACTIVE_SUBJECT] = (float)subject_id;
    params[O_ROUTE_DIRECTION] = (float)route_direction;

    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}

void menu_params_save_path(void)
{
    memset(params, 0, sizeof(params));
    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);
    }

    params[O_BASIC_SPEED] = basic_speed;
    memcpy(&params[O_PID_LEFT], &pid_left, sizeof(PosPID));
    memcpy(&params[O_PID_RIGHT], &pid_right, sizeof(PosPID));
    params[O_MAX_SPEED] = (float)max_target_speed;
    params[O_MIN_SPEED] = (float)min_target_speed;
    params[O_DECEL_STEP] = (float)DECEL_STEP;
    params[O_ACCEL_STEP] = (float)ACCEL_STEP;
    params[O_FORK_ENTRY_SHRINK] = (float)fork_entry_shrink_percent;
    params[O_FORK_EXIT_SHRINK] = (float)fork_exit_shrink_percent;
    params[O_FORK_ENTRY_MARGIN] = (float)fork_entry_margin_ticks;
    params[O_FORK_EXIT_MARGIN] = (float)fork_exit_margin_ticks;
    params[O_FORK_CORE_MIN] = (float)fork_hard_core_min_ticks;
    params[O_PARAM_LAYOUT_VERSION] = PARAM_LAYOUT_VERSION;

    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}

void menu_params_save_tuning_preset(void)
{
    memset(params, 0, sizeof(params));
    if (flash_check(0, PARAM_PAGE) == 1)
    {
        flash_read_page(0, PARAM_PAGE, (uint32 *)params, PARAM_WORD_COUNT);
    }

    params[O_BASIC_SPEED] = basic_speed;
    memcpy(&params[O_TURNPID], &turnPID, sizeof(SteeringPID));
    params[O_STEER_KP2] = steer_Kp2;
    params[O_TD_FREQ] = td_freq;
    params[O_TD_DAMP] = td_damp;
    params[O_DEAD_ZONE] = dead_zone;
    params[O_FORK_FRONT] = (float)fork_front_point;
    params[O_NORMAL_FRONT] = (float)normal_front_point;
    params[O_FORK_EXIT_LOOKBACK] = (float)fork_exit_lookback_points;
    params[O_PARAM_LAYOUT_VERSION] = PARAM_LAYOUT_VERSION;
    tuning_presets_store(&params[O_PRESET_BASE]);

    flash_write_page(0, PARAM_PAGE, (const uint32 *)params, PARAM_WORD_COUNT);
}
