#include "menu_tree.h"

#include "menu.h"
#include "menu_pages.h"
#include "menu_params.h"
#include "route_progress.h"
#include "run_features.h"
#include "tuning_presets.h"
#include "zf_common_headfile.h"
#include "zf_device_ips114.h"

#include <stdio.h>

extern PosPID pid_left;
extern PosPID pid_right;
extern SteeringPID turnPID;
extern int min_target_speed;
extern int max_target_speed;
extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;
extern float GZ;

static const MenuPage set_page;
static const MenuPage prm_page;
static const MenuPage preset_page;
static const MenuPage preset_editor_page;
static const MenuPage steering_page;
static const MenuPage speed_page;
static const MenuPage img_page;
static const MenuPage subject_page;
static const MenuPage path_page;
static const MenuPage path_map_page;
static const MenuPage path_velocity_page;
static const MenuPage path_assist_page;
static const MenuPage path_status_page;
static const MenuPage cal_page;
static const MenuPage imu_status_page;
static const MenuPage encoder_page;

static char g_preset_labels[TUNING_PRESET_COUNT][16];
static TuningPreset g_preset_edit;
static uint8 g_preset_edit_index = 0U;

static void menu_tree_show_saved(uint32 delay_ms)
{
    ips114_show_string(176, 112, "SAVE");
    system_delay_ms(delay_ms);
}

static void menu_tree_show_message(const char *message, uint32 delay_ms)
{
    ips114_show_string(160, 112, "          ");
    ips114_show_string(160, 112, message);
    system_delay_ms(delay_ms);
}

static void menu_tree_save_steering(MenuContext *context)
{
    (void)context;
    menu_params_save_steering();
    menu_tree_show_saved(500U);
}

static void menu_tree_save_speed(MenuContext *context)
{
    (void)context;
    menu_params_save_speed();
    menu_tree_show_saved(500U);
}

static void menu_tree_save_image(MenuContext *context)
{
    (void)context;
    menu_params_save_image();
    menu_tree_show_saved(500U);
}

static void menu_tree_save_path(MenuContext *context)
{
    (void)context;
    menu_params_save_path();
    menu_tree_show_saved(500U);
}

static uint8_t menu_tree_get_record(void)
{
    return record_enable;
}

static uint8_t menu_tree_get_speed_decision(void)
{
    return speed_decision_enable;
}

static uint8_t menu_tree_get_fork_correction(void)
{
    return fork_correction_enable;
}

static uint8_t menu_tree_get_drs(void)
{
    return drs_enable;
}

static void menu_tree_toggle_record(void)
{
    run_features_set_record((uint8_t)!record_enable);
}

static void menu_tree_toggle_speed_decision(void)
{
    run_features_set_speed_decision((uint8_t)!speed_decision_enable);
}

static void menu_tree_toggle_fork_correction(void)
{
    run_features_set_fork_correction((uint8_t)!fork_correction_enable);
}

static void menu_tree_toggle_drs(void)
{
    run_features_set_drs((uint8_t)!drs_enable);
}

static void menu_tree_load_map(MenuContext *context)
{
    TrajectoryResult result;

    (void)context;
    result = Load_Trajectory_From_Flash();
    if (result == TRAJECTORY_OK) {
        menu_tree_show_message("LOADED", 500U);
    } else {
        menu_tree_show_message("NO MAP", 500U);
    }
}

static void menu_tree_save_map(MenuContext *context)
{
    TrajectoryResult result;

    (void)context;
    result = Save_Trajectory_To_Flash();
    if (result == TRAJECTORY_OK) {
        menu_tree_show_message("SAVED", 500U);
    } else {
        menu_tree_show_message("INVALID", 500U);
    }
}

static void menu_tree_replan_velocity(MenuContext *context)
{
    (void)context;
    if (INS_Generate_Velocity_Table(min_target_speed, max_target_speed)) {
        (void)route_progress_rebuild_map();
        menu_tree_show_message("PLANNED", 500U);
    } else {
        menu_tree_show_message("INVALID", 500U);
    }
}

static uint8_t menu_tree_get_motor_awake(void)
{
    return (uint8_t)gpio_get_level(NSLEEP_PIN);
}

static void menu_tree_toggle_motor(void)
{
    if (gpio_get_level(NSLEEP_PIN)) {
        NSLEEP_Sleep();
    } else {
        NSLEEP_Wake();
    }
}

static void menu_tree_select_route(MenuContext *context, uint8_t value)
{
    subject_id = value < 2U ? 2U : 3U;
    route_direction = (value & 1U) == 0U
        ? ROUTE_DIRECTION_FORWARD : ROUTE_DIRECTION_REVERSE;
    menu_params_save_subject();
    menu_tree_show_saved(300U);
    menu_nav_request_redraw(context);
}

static void menu_tree_select_prelim_forward(MenuContext *context)
{
    menu_tree_select_route(context, 0U);
}

static void menu_tree_select_prelim_reverse(MenuContext *context)
{
    menu_tree_select_route(context, 1U);
}

static void menu_tree_select_final_forward(MenuContext *context)
{
    menu_tree_select_route(context, 2U);
}

static void menu_tree_select_final_reverse(MenuContext *context)
{
    menu_tree_select_route(context, 3U);
}

static void menu_tree_subject_enter(MenuContext *context)
{
    uint8_t cursor = subject_id == 2U ? 0U : 2U;

    if (route_direction == ROUTE_DIRECTION_REVERSE)
    {
        cursor++;
    }
    menu_nav_set_cursor(context, cursor);
}

static void menu_tree_refresh_preset_labels(void)
{
    uint8_t i;

    for (i = 0U; i < TUNING_PRESET_COUNT; i++)
    {
        if (tuning_preset_is_configured(i))
        {
            (void)snprintf(g_preset_labels[i], sizeof(g_preset_labels[i]),
                           "SPD %d", (int)tuning_preset_basic_speed(i));
        }
        else
        {
            (void)snprintf(g_preset_labels[i], sizeof(g_preset_labels[i]),
                           "SPD ---");
        }
    }
}

static void menu_tree_preset_enter(MenuContext *context)
{
    menu_tree_refresh_preset_labels();
    menu_nav_request_redraw(context);
}

static void menu_tree_apply_preset(MenuContext *context)
{
    MenuFrame *frame = menu_nav_current_frame(context);
    TuningPresetResult result;

    if (frame == NULL || frame->cursor >= TUNING_PRESET_COUNT)
    {
        menu_tree_show_message("INVALID", 500U);
        return;
    }

    result = tuning_preset_apply(frame->cursor);
    if (result == TUNING_PRESET_APPLIED)
    {
        menu_tree_show_message("APPLIED", 500U);
    }
    else if (result == TUNING_PRESET_EMPTY)
    {
        menu_tree_show_message("EMPTY", 500U);
    }
    else
    {
        menu_tree_show_message("INVALID", 500U);
    }
    menu_nav_request_redraw(context);
}

static void menu_tree_preset_editor_enter(MenuContext *context)
{
    menu_nav_set_cursor(context, 1U);
}

static void menu_tree_save_edited_preset(MenuContext *context)
{
    TuningPresetResult result = tuning_preset_save_and_apply(
        g_preset_edit_index,
        &g_preset_edit);

    if (result != TUNING_PRESET_APPLIED)
    {
        menu_tree_show_message("INVALID", 500U);
        menu_nav_request_redraw(context);
        return;
    }

    menu_tree_refresh_preset_labels();
    menu_tree_show_message("APPLIED", 500U);
    menu_nav_go_back(context);
}

static void menu_tree_preset_key(MenuContext *context, MenuKey key)
{
    MenuFrame *frame = menu_nav_current_frame(context);
    uint8 cursor;

    if (frame == NULL)
    {
        return;
    }

    cursor = frame->cursor;
    if (key == MENU_KEY_UP)
    {
        cursor = cursor == 0U
            ? (uint8)(TUNING_PRESET_COUNT - 1U)
            : (uint8)(cursor - 1U);
        menu_nav_set_cursor(context, cursor);
    }
    else if (key == MENU_KEY_DOWN)
    {
        cursor = (uint8)((cursor + 1U) % TUNING_PRESET_COUNT);
        menu_nav_set_cursor(context, cursor);
    }
    else if (key == MENU_KEY_PRESS)
    {
        menu_tree_apply_preset(context);
    }
    else if (key == MENU_KEY_RIGHT)
    {
        g_preset_edit_index = cursor;
        tuning_preset_begin_edit(g_preset_edit_index, &g_preset_edit);
        menu_nav_open_page(context, &preset_editor_page);
    }
}

static void menu_tree_status_points(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d", Total_Path_Points);
}

static void menu_tree_status_events(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d", g_map_event_count);
}

static void menu_tree_status_fork_events(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d", route_progress_event_count);
}

static void menu_tree_status_fork_index(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d",
                   route_progress_current_event_index);
}

static void menu_tree_status_phase(char *buffer, size_t buffer_size)
{
    static const char *const labels[] = {
        "OUT", "ENTRY", "HARD", "EXIT", "DONE"
    };
    int phase = (int)route_progress_phase;
    const char *label = (phase >= 0 && phase < 5) ? labels[phase] : "ERR";
    (void)snprintf(buffer, buffer_size, "%s", label);
}

static void menu_tree_status_mileage(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d", route_progress_mileage_snapshot);
}

static void menu_tree_status_event_boundary(
    char *buffer,
    size_t buffer_size,
    int boundary_index)
{
    const ForkCorrectionEvent *event = route_progress_current_event();
    int value;

    if (event == NULL) {
        (void)snprintf(buffer, buffer_size, "--");
        return;
    }
    if (boundary_index == 0) value = event->window_start;
    else if (boundary_index == 1) value = event->hard_start;
    else if (boundary_index == 2) value = event->hard_end;
    else value = event->window_end;
    (void)snprintf(buffer, buffer_size, "%d", value);
}

static void menu_tree_status_window_start(char *buffer, size_t buffer_size)
{
    menu_tree_status_event_boundary(buffer, buffer_size, 0);
}

static void menu_tree_status_hard_start(char *buffer, size_t buffer_size)
{
    menu_tree_status_event_boundary(buffer, buffer_size, 1);
}

static void menu_tree_status_hard_end(char *buffer, size_t buffer_size)
{
    menu_tree_status_event_boundary(buffer, buffer_size, 2);
}

static void menu_tree_status_window_end(char *buffer, size_t buffer_size)
{
    menu_tree_status_event_boundary(buffer, buffer_size, 3);
}

static const char *menu_tree_source_label(ForkCorrectionSource source)
{
    if (source == FORK_CORR_SOURCE_VISION) return "VIS";
    if (source == FORK_CORR_SOURCE_FORCED) return "FORCE";
    if (source == FORK_CORR_SOURCE_NORMAL_EXIT) return "NORMAL";
    return "--";
}

static void menu_tree_status_enter_source(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%s",
                   menu_tree_source_label(route_progress_enter_source));
}

static void menu_tree_status_exit_source(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%s",
                   menu_tree_source_label(route_progress_exit_source));
}

static void menu_tree_status_fork_sync(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d>%d",
                   route_progress_sync_fork_before,
                   route_progress_sync_fork_after);
}

static void menu_tree_status_angle_sync(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d>%d",
                   route_progress_sync_angle_before,
                   route_progress_sync_angle_after);
}

static void menu_tree_status_build(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%d",
                   (int)route_progress_build_result);
}

static void menu_tree_path_status_enter(MenuContext *context)
{
    (void)context;
    if (trajectory_map_valid && Total_Path_Points > 0) {
        (void)route_progress_rebuild_map();
    }
}

static void menu_tree_status_gyro_x(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.3f", gyro_offset_x);
}

static void menu_tree_status_gyro_y(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.3f", gyro_offset_y);
}

static void menu_tree_status_gyro_z(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.3f", gyro_offset_z);
}

static void menu_tree_status_yaw(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.1f", eulerAngle.yaw);
}

static void menu_tree_status_pitch(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.1f", eulerAngle.pitch);
}

static void menu_tree_status_roll(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.1f", eulerAngle.roll);
}

static void menu_tree_status_gz(char *buffer, size_t buffer_size)
{
    (void)snprintf(buffer, buffer_size, "%+7.3f", GZ);
}

static const MenuItem steering_items[] = {
    { .label = "bKp", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &turnPID.Kp, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "bKd", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &turnPID.Kd, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "Kp2", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &steer_Kp2, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "Gkd", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &turnPID.GKD, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "tdF", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &td_freq, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "tdD", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &td_damp, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "dZn", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &dead_zone, 0.0f, 2000.0f, 2U, NULL } }
};

static const MenuItem speed_items[] = {
    { .label = "spd", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &basic_speed, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "lkp", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &pid_left.Kp, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "lki", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &pid_left.Ki, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "rkp", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &pid_right.Kp, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "rki", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &pid_right.Ki, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "Kff", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &K_forward, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "Fan", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &negative_pressure_percent, 0.0f, 2000.0f, 2U, NULL } }
};

static const MenuItem preset_editor_items[] = {
    { .label = "Save+Apply", .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_save_edited_preset },
    { .label = "bKp", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.turn_kp, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "bKd", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.turn_kd, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "Kp2", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.steer_kp2, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "Gkd", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.gyro_kd, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "tdF", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.td_freq, 1.0f, 100.0f, 2U, NULL } },
    { .label = "tdD", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.td_damp, 0.5f, 2.0f, 2U, NULL } },
    { .label = "dZn", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.dead_zone, 0.0f, 50.0f, 2U, NULL } },
    { .label = "spd", .type = MENU_ITEM_FLOAT,
      .data.float_value = { &g_preset_edit.basic_speed, 0.0f, 2000.0f, 2U, NULL } },
    { .label = "nFwd", .type = MENU_ITEM_INT,
      .data.int_value = { &g_preset_edit.normal_front_point, 10, 300 } },
    { .label = "fFwd", .type = MENU_ITEM_INT,
      .data.int_value = { &g_preset_edit.fork_front_point, 10, 300 } },
    { .label = "bBack", .type = MENU_ITEM_INT,
      .data.int_value = {
          &g_preset_edit.fork_exit_lookback_points, 0, 40 } }
};

static const MenuItem preset_items[] = {
    { .label = g_preset_labels[0], .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_apply_preset },
    { .label = g_preset_labels[1], .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_apply_preset },
    { .label = g_preset_labels[2], .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_apply_preset },
    { .label = g_preset_labels[3], .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_apply_preset },
    { .label = g_preset_labels[4], .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_apply_preset }
};

static const MenuItem prm_items[] = {
    { .label = "Preset", .type = MENU_ITEM_SUBMENU, .data.page = &preset_page },
    { .label = "Steering", .type = MENU_ITEM_SUBMENU, .data.page = &steering_page },
    { .label = "Speed", .type = MENU_ITEM_SUBMENU, .data.page = &speed_page }
};

static const MenuItem subject_items[] = {
    { .label = "Pre Fwd", .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_select_prelim_forward },
    { .label = "Pre Rev", .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_select_prelim_reverse },
    { .label = "Final Fwd", .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_select_final_forward },
    { .label = "Final Rev", .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_select_final_reverse }
};

static const MenuItem path_map_items[] = {
    { .label = "Record", .type = MENU_ITEM_TOGGLE,
      .data.toggle = { menu_tree_get_record, menu_tree_toggle_record } },
    { .label = "Speed", .type = MENU_ITEM_TOGGLE,
      .data.toggle = { menu_tree_get_speed_decision,
                       menu_tree_toggle_speed_decision } },
    { .label = "Fork", .type = MENU_ITEM_TOGGLE,
      .data.toggle = { menu_tree_get_fork_correction,
                       menu_tree_toggle_fork_correction } },
    { .label = "DRS", .type = MENU_ITEM_TOGGLE,
      .data.toggle = { menu_tree_get_drs, menu_tree_toggle_drs } },
    { .label = "Load Map", .type = MENU_ITEM_ACTION, .data.action = menu_tree_load_map },
    { .label = "Save Map", .type = MENU_ITEM_ACTION, .data.action = menu_tree_save_map },
    { .label = "Re-Plan Vel", .type = MENU_ITEM_ACTION,
      .data.action = menu_tree_replan_velocity }
};

static const MenuItem path_velocity_items[] = {
    { .label = "MaxV", .type = MENU_ITEM_INT,
      .data.int_value = { &max_target_speed, 0, 2000 } },
    { .label = "MinV", .type = MENU_ITEM_INT,
      .data.int_value = { &min_target_speed, 0, 2000 } },
    { .label = "Decel", .type = MENU_ITEM_INT,
      .data.int_value = { &DECEL_STEP, TRAJECTORY_STEP_MIN,
                          TRAJECTORY_STEP_MAX } },
    { .label = "Accel", .type = MENU_ITEM_INT,
      .data.int_value = { &ACCEL_STEP, TRAJECTORY_STEP_MIN,
                          TRAJECTORY_STEP_MAX } }
};

static const MenuItem path_assist_items[] = {
    { .label = "InShrink", .type = MENU_ITEM_INT,
      .data.int_value = { &fork_entry_shrink_percent, 0, 45, 5 } },
    { .label = "OutShrink", .type = MENU_ITEM_INT,
      .data.int_value = { &fork_exit_shrink_percent, 0, 45, 5 } },
    { .label = "InMargin", .type = MENU_ITEM_INT,
      .data.int_value = { &fork_entry_margin_ticks, 0, 10000, 500 } },
    { .label = "OutMargin", .type = MENU_ITEM_INT,
      .data.int_value = { &fork_exit_margin_ticks, 0, 10000, 500 } },
    { .label = "CoreMin", .type = MENU_ITEM_INT,
      .data.int_value = { &fork_hard_core_min_ticks, 0, 10000, 500 } }
};

static const MenuItem path_status_items[] = {
    { .label = "Pts", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_points },
    { .label = "Evts", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_events },
    { .label = "ForkEv", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_fork_events },
    { .label = "FIdx", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_fork_index },
    { .label = "Phase", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_phase },
    { .label = "Mil", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_mileage },
    { .label = "WinS", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_window_start },
    { .label = "HardS", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_hard_start },
    { .label = "HardE", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_hard_end },
    { .label = "WinE", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_window_end },
    { .label = "Enter", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_enter_source },
    { .label = "Exit", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_exit_source },
    { .label = "FSync", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_fork_sync },
    { .label = "ASync", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_angle_sync },
    { .label = "Build", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_build }
};

static const MenuItem path_items[] = {
    { .label = "Map", .type = MENU_ITEM_SUBMENU, .data.page = &path_map_page },
    { .label = "Velocity", .type = MENU_ITEM_SUBMENU, .data.page = &path_velocity_page },
    { .label = "Assist", .type = MENU_ITEM_SUBMENU, .data.page = &path_assist_page },
    { .label = "Status", .type = MENU_ITEM_SUBMENU, .data.page = &path_status_page }
};

static const MenuItem imu_status_items[] = {
    { .label = "Ox", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_gyro_x },
    { .label = "Oy", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_gyro_y },
    { .label = "Oz", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_gyro_z },
    { .label = "Pitch", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_pitch },
    { .label = "Roll", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_roll },
    { .label = "Yaw", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_yaw },
    { .label = "GZ", .type = MENU_ITEM_STATUS, .data.status = menu_tree_status_gz }
};

static const MenuItem cal_items[] = {
    { .label = "IMU Status", .type = MENU_ITEM_SUBMENU, .data.page = &imu_status_page },
    { .label = "Gyro Calib", .type = MENU_ITEM_ACTION,
      .data.action = menu_page_gyro_calibrate },
    { .label = "Encoder Check", .type = MENU_ITEM_CUSTOM, .data.page = &encoder_page }
};

static const MenuItem root_items[] = {
    { .label = "Set", .type = MENU_ITEM_CUSTOM, .data.page = &set_page },
    { .label = "PRM", .type = MENU_ITEM_SUBMENU, .data.page = &prm_page },
    { .label = "IMG", .type = MENU_ITEM_CUSTOM, .data.page = &img_page },
    { .label = "SUB", .type = MENU_ITEM_SUBMENU, .data.page = &subject_page },
    { .label = "Path", .type = MENU_ITEM_SUBMENU, .data.page = &path_page },
    { .label = "Cal", .type = MENU_ITEM_SUBMENU, .data.page = &cal_page },
    { .label = "MOT", .type = MENU_ITEM_TOGGLE,
      .data.toggle = { menu_tree_get_motor_awake, menu_tree_toggle_motor } }
};

static const MenuPage set_page = {
    "Set", NULL, 0U, 0U, NULL, NULL,
    menu_page_set_key, NULL, menu_page_set_render
};

static const MenuPage steering_page = {
    "PRM / Steering", steering_items,
    (uint8_t)(sizeof(steering_items) / sizeof(steering_items[0])),
    0U, NULL, menu_tree_save_steering, NULL, NULL, NULL
};

static const MenuPage preset_page = {
    "Preset P:Use R:Edit", preset_items,
    (uint8_t)(sizeof(preset_items) / sizeof(preset_items[0])),
    0U, menu_tree_preset_enter, NULL, menu_tree_preset_key, NULL, NULL
};

static const MenuPage preset_editor_page = {
    "Preset Edit", preset_editor_items,
    (uint8_t)(sizeof(preset_editor_items) / sizeof(preset_editor_items[0])),
    0U, menu_tree_preset_editor_enter, NULL, NULL, NULL, NULL
};

static const MenuPage speed_page = {
    "PRM / Speed", speed_items,
    (uint8_t)(sizeof(speed_items) / sizeof(speed_items[0])),
    0U, NULL, menu_tree_save_speed, NULL, NULL, NULL
};

static const MenuPage prm_page = {
    "PRM", prm_items, (uint8_t)(sizeof(prm_items) / sizeof(prm_items[0])),
    0U, NULL, NULL, NULL, NULL, NULL
};

static const MenuPage img_page = {
    "IMG", NULL, 0U, 1U, menu_page_img_enter, menu_tree_save_image,
    menu_page_img_key, menu_page_img_tick, menu_page_img_render
};

static const MenuPage subject_page = {
    "SUB", subject_items,
    (uint8_t)(sizeof(subject_items) / sizeof(subject_items[0])),
    0U, menu_tree_subject_enter, NULL, NULL, NULL, NULL
};

static const MenuPage path_map_page = {
    "Path / Map", path_map_items,
    (uint8_t)(sizeof(path_map_items) / sizeof(path_map_items[0])),
    0U, NULL, NULL, NULL, NULL, NULL
};

static const MenuPage path_velocity_page = {
    "Path / Velocity", path_velocity_items,
    (uint8_t)(sizeof(path_velocity_items) / sizeof(path_velocity_items[0])),
    0U, NULL, NULL, NULL, NULL, NULL
};

static const MenuPage path_assist_page = {
    "Path / Assist", path_assist_items,
    (uint8_t)(sizeof(path_assist_items) / sizeof(path_assist_items[0])),
    0U, NULL, NULL, NULL, NULL, NULL
};

static const MenuPage path_status_page = {
    "Path / Status", path_status_items,
    (uint8_t)(sizeof(path_status_items) / sizeof(path_status_items[0])),
    1U, menu_tree_path_status_enter, NULL, NULL, NULL, NULL
};

static const MenuPage path_page = {
    "Path", path_items, (uint8_t)(sizeof(path_items) / sizeof(path_items[0])),
    0U, NULL, menu_tree_save_path, NULL, NULL, NULL
};

static const MenuPage imu_status_page = {
    "Cal / IMU", imu_status_items,
    (uint8_t)(sizeof(imu_status_items) / sizeof(imu_status_items[0])),
    1U, NULL, NULL, NULL, NULL, NULL
};

static const MenuPage encoder_page = {
    "Encoder", NULL, 0U, 1U, NULL, NULL,
    NULL, NULL, menu_page_encoder_render
};

static const MenuPage cal_page = {
    "Cal", cal_items, (uint8_t)(sizeof(cal_items) / sizeof(cal_items[0])),
    0U, NULL, NULL, NULL, NULL, NULL
};

static const MenuPage root_page = {
    "MENU", root_items, (uint8_t)(sizeof(root_items) / sizeof(root_items[0])),
    0U, NULL, NULL, NULL, NULL, NULL
};

const MenuPage *menu_tree_root_page(void)
{
    return &root_page;
}
