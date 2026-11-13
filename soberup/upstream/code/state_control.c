/**************************************************************
 * 文件名: state_control.c
 * 说明: 三叉状态、直角状态以及基于里程窗口的三叉裁决。
 **************************************************************/

#include "element_line.h"
#include "state_control.h"
#include "line.h"
#include "zf_device_ips114.h"
#include "maze.h"
#include "MahonyAHRS.h"
#include "LED.h"
#include "menu.h"
#include "motor.h"
#include "route_config.h"
#include "route_progress.h"
#include "run_features.h"
#include "trajectory_policy.h"

#include <math.h>

volatile state car_state = normal;
volatile element_state line_state = none;

volatile float current_angle = 0.0f;
int count = 0;
static int straight_count = 0;
static int fork_turn_frames = 0;
static int fork_turn_lost_count = 0;
static int fork_reentry_block_frames = 0;
static int state_stop_sent = 0;
static volatile float fork_entry_yaw = 0.0f;
static volatile float fork_imu_exit_angle = 0.0f;
static volatile uint8_t fork_imu_exit_allowed = 0U;
static volatile uint8_t turn_end_damping_active = 0U;
static uint8_t fork_entry_exit_pair = FORK_EXIT_PAIR_INVALID;
static int fork_entry_left_ticks = 0;
static int fork_entry_right_ticks = 0;
static uint8_t fork_yaw_protect_consecutive = 0U;

#define FORK_EXIT_ANGLE 60                    // 转弯三叉的陀螺仪兜底退出角度
#define FORK_EXIT_PREPARE_ANGLE 40            // 开始检测转弯三叉出口的最小转角
#define FORK_EXIT_LOST_CONFIRM_FRAMES 3       // 转弯出口视觉丢失的确认帧数
#define FORK_PRELIM_FORWARD_BLIND_INDEX 40    // Pre Fwd 循右线等待左右三叉的叉号
#define FORK_GUIDE_MAX_ANGLE 40               // 转弯三叉保持路径引导的最大转角
#define FORK_ENTRY_DEBOUNCE 0                 // 视觉入叉的防抖帧数，0 表示不防抖
#define FORK_REENTRY_DELAY_FRAMES 0           // 退出三叉后禁止再次入叉的帧数
#define TURN_END_DAMPING_ANGLE 40.0f          // 启用转弯末段阻尼的最小转角
#define TURN_END_DAMPING_SCALE 2.0f           // 转弯末段的陀螺仪阻尼项倍率
#define ENCODER_LEFT_MM_PER_TICK 0.017526346f // 左编码器每个计数对应的毫米数
#define ENCODER_RIGHT_MM_PER_TICK 0.017562399f // 右编码器每个计数对应的毫米数
#define STRAIGHT_FORK_EXIT_DISTANCE_MM 400.0f // Record 模式直行三叉的里程退出距离
#define FORK_YAW_PROTECT_ERROR_DEG 90.0f      // 入叉航向与配置参考偏差超过此角度视为走错
#define FORK_YAW_PROTECT_CONFIRM_FORKS 2U     // 连续多少个三叉超偏差才停车

extern Point out_L;
extern Point out_R;
extern int dis_L;
extern int dis_R;

int current_fork_type = FORK_TYPE_TURN;
int angle_num = 0;
int fork_entry_element_count = 0;
int fork_normal_entry_count = 0;
int fork_forced_entry_count = 0;
int fork_normal_exit_count = 0;
int fork_forced_exit_count = 0;

static float yaw_angle_difference(float yaw_a, float yaw_b)
{
    float difference = fabsf(yaw_a - yaw_b);

    if (difference > 180.0f)
    {
        difference = 360.0f - difference;
    }

    return difference;
}

static void state_refresh_turn_damping_isr(void)
{
    float entry_yaw;

    if (car_state == fork_turn)
    {
        entry_yaw = fork_entry_yaw;
    }
    else if (line_state == angle)
    {
        entry_yaw = current_angle;
    }
    else
    {
        turn_end_damping_active = 0U;
        return;
    }

    turn_end_damping_active = (uint8_t)(
        yaw_angle_difference(get_yaw(), entry_yaw) >=
        TURN_END_DAMPING_ANGLE);
}

float state_get_turn_damping_scale(void)
{
    return turn_end_damping_active ? TURN_END_DAMPING_SCALE : 1.0f;
}

void state_imu_exit_check_isr(void)
{
    float angle_difference;

    state_refresh_turn_damping_isr();

    if (car_state != fork_turn || !fork_imu_exit_allowed)
    {
        return;
    }

    angle_difference = yaw_angle_difference(get_yaw(), fork_entry_yaw);
    if (angle_difference > FORK_EXIT_ANGLE)
    {
        /* 中断内只锁存角度并切出转弯态，完整收尾仍由 CPU1 执行。 */
        fork_imu_exit_angle = angle_difference;
        car_state = fork_exit_pending;
    }
}

int fork_turn_guide_is_active(void)
{
    if (car_state != fork_turn)
    {
        return 0;
    }
    return yaw_angle_difference(get_yaw(), current_angle) <=
        FORK_GUIDE_MAX_ANGLE;
}

static void fork_num_increase(void)
{
    fork_num++;
    if (fork_num >= FORK_CONFIG_CAPACITY)
    {
        fork_num = FORK_CONFIG_CAPACITY - 1;
    }
}

static void state_clear_exit_evidence(void)
{
    straight_count = 0;
    fork_turn_lost_count = 0;
}

static void state_show_fork_led(int fork_index)
{
    if ((fork_index & 1) == 0)
    {
        LED1_on();
        LED2_off();
    }
    else
    {
        LED1_off();
        LED2_on();
    }
}

static void state_clear_fork_leds(void)
{
    LED1_off();
    LED2_off();
}

static uint8_t state_straight_fork_distance_reached(void)
{
    int left_ticks = dis_L - fork_entry_left_ticks;
    int right_ticks = dis_R - fork_entry_right_ticks;
    float left_distance_mm =
        (float)left_ticks * ENCODER_LEFT_MM_PER_TICK;
    float right_distance_mm =
        (float)right_ticks * ENCODER_RIGHT_MM_PER_TICK;

    return (uint8_t)(record_enable &&
        (left_distance_mm + right_distance_mm) * 0.5f >=
        STRAIGHT_FORK_EXIT_DISTANCE_MM);
}

/* 角度观测器：入叉航向与配置参考航向持续超偏即停车，防止走错方向。 */
static void state_check_fork_entry_yaw_protection(int config_index)
{
    float yaw_error;
    int expected_yaw;

    if (!route_fork_entry_angle_valid(
            subject_id, route_direction, config_index))
    {
        return;
    }

    expected_yaw = route_get_fork_entry_angle_deg(
        subject_id, route_direction, config_index);
    yaw_error = yaw_angle_difference(fork_entry_yaw, (float)expected_yaw);

    if (yaw_error > FORK_YAW_PROTECT_ERROR_DEG)
    {
        if (fork_yaw_protect_consecutive <
            FORK_YAW_PROTECT_CONFIRM_FORKS)
        {
            fork_yaw_protect_consecutive++;
        }
        if (fork_yaw_protect_consecutive >=
            FORK_YAW_PROTECT_CONFIRM_FORKS)
        {
            car_stop(STOP_REASON_YAW_PROTECTION);
        }
    }
    else
    {
        fork_yaw_protect_consecutive = 0U;
    }
}

static void state_enter_current_fork(ForkCorrectionSource source)
{
    int config_index = route_progress_config_fork_index();

    if (source == FORK_CORR_SOURCE_FORCED)
    {
        fork_forced_entry_count++;
    }
    else
    {
        fork_normal_entry_count++;
    }

    if (!route_fork_index_valid(config_index))
    {
        config_index = fork_num;
    }
    current_fork_type = route_get_fork_type(
        subject_id, route_direction, config_index);
    if (source == FORK_CORR_SOURCE_VISION)
    {
        fork_entry_exit_pair = route_classify_fork_exit_pair(
            fork_dbg_boundary_L,
            fork_dbg_boundary_R);
    }
    else
    {
        fork_entry_exit_pair = route_get_fork_exit_pair(
            subject_id, route_direction, config_index);
    }
    fork_entry_yaw = get_yaw();
    turn_end_damping_active = 0U;
    fork_mileage_report_begin();
    fork_imu_exit_allowed = (uint8_t)(
        current_fork_type == FORK_TYPE_TURN && !fork_correction_enable);
    fork_turn_lost_count = 0;

    if (current_fork_type == FORK_TYPE_STRAIGHT)
    {
        fork_entry_left_ticks = dis_L;
        fork_entry_right_ticks = dis_R;
        car_state = fork_straight;
        straight_count = 0;
    }
    else
    {
        car_state = fork_turn;
        current_angle = get_yaw();
        fork_turn_frames = 0;
    }

    fork_reentry_block_frames = 0;
    count = 0;
    state_show_fork_led(config_index);

    if (fork_correction_enable)
    {
        route_progress_on_enter(source);
    }
    if (source == FORK_CORR_SOURCE_VISION &&
        run_features_map_enabled())
    {
        INS_Relocalize_At_Flag(
            (int)trajectory_encode_fork(config_index));
    }
    state_check_fork_entry_yaw_protection(config_index);
}

static void state_finish_current_fork(ForkCorrectionSource source)
{
    float angle_difference = car_state == fork_exit_pending
        ? fork_imu_exit_angle
        : yaw_angle_difference(get_yaw(), fork_entry_yaw);

    fork_exit_mileage_report_capture(
        fork_num,
        (int)(angle_difference + 0.5f));

    if (source == FORK_CORR_SOURCE_FORCED)
    {
        fork_forced_exit_count++;
    }
    else
    {
        fork_normal_exit_count++;
    }

    car_state = normal;
    turn_end_damping_active = 0U;
    state_clear_fork_leds();
    fork_imu_exit_allowed = 0U;
    state_clear_exit_evidence();
    fork_turn_frames = 0;
    fork_reentry_block_frames = FORK_REENTRY_DELAY_FRAMES;
    count = 0;

    if (fork_correction_enable)
    {
        route_progress_on_exit(source);
    }
    else
    {
        fork_num_increase();
    }
    current_fork_type = route_get_fork_type(
        subject_id, route_direction, fork_num);
}

void state_reset_run(void)
{
    car_state = normal;
    line_state = none;
    current_angle = 0.0f;
    count = 0;
    straight_count = 0;
    fork_turn_frames = 0;
    fork_turn_lost_count = 0;
    fork_reentry_block_frames = 0;
    state_stop_sent = 0;
    fork_entry_left_ticks = 0;
    fork_entry_right_ticks = 0;
    fork_yaw_protect_consecutive = 0U;
    fork_num = 0;
    angle_num = 0;
    fork_normal_entry_count = 0;
    fork_forced_entry_count = 0;
    fork_normal_exit_count = 0;
    fork_forced_exit_count = 0;
    fork_imu_exit_angle = 0.0f;
    fork_imu_exit_allowed = 0U;
    turn_end_damping_active = 0U;
    fork_entry_exit_pair = FORK_EXIT_PAIR_INVALID;
    current_fork_type = route_get_fork_type(
        subject_id, route_direction, fork_num);
    state_clear_fork_leds();
}

void state_update(void)
{
    float end_angle = 0.0f;
    float angle_difference = 0.0f;
    int element_count;
    int pair_valid;
    int config_index;
    uint8_t detected_exit_pair;
    int vision_entry_condition;
    int fork40_blind_mode;
    int fork40_left_right_detected;
    int exit_pair_required;
    int exit_pair_matches;

    if (car_state == fork_exit_pending)
    {
        state_finish_current_fork(FORK_CORR_SOURCE_NORMAL_EXIT);
        return;
    }

    if (fork_correction_enable &&
        route_progress_exit_window_just_opened())
    {
        state_clear_exit_evidence();
    }

    element_count = count_valid_exits();
    pair_valid = element_count == 1 &&
        out_L.x && out_L.y && out_R.x && out_R.y;
    detected_exit_pair = pair_valid
        ? route_classify_fork_exit_pair(
            fork_dbg_boundary_L,
            fork_dbg_boundary_R)
        : FORK_EXIT_PAIR_INVALID;
    fork_entry_element_count = element_count;

    config_index = route_progress_config_fork_index();
    if (!route_fork_index_valid(config_index))
    {
        config_index = fork_num;
    }
    current_fork_type = route_get_fork_type(
        subject_id, route_direction, config_index);

    /* 穿出表生效：该叉配置了穿出对时视觉入口必须匹配，表空(INVALID)时不强制。 */
    exit_pair_required = route_get_fork_exit_pair(
        subject_id, route_direction, config_index) != FORK_EXIT_PAIR_INVALID;
    exit_pair_matches = pair_valid &&
        route_fork_exit_pair_matches(
            subject_id,
            route_direction,
            config_index,
            fork_dbg_boundary_L,
            fork_dbg_boundary_R);

    fork_imu_exit_allowed = (uint8_t)(
        car_state == fork_turn &&
        (!fork_correction_enable || route_progress_normal_exit_allowed()));

    if (fork_num >= route_get_stop_fork_num(
            subject_id, route_direction))
    {
        if (!state_stop_sent)
        {
            state_stop_sent = 1;
            car_stop(STOP_REASON_ROUTE_END);
        }
        return;
    }

    vision_entry_condition = element_count == 1 &&
        R_point >= 70 && L_point >= 70 && trend.slope > 80;
    /* Pre Fwd 叉40盲区：Fork关闭时持续循右线，仅接受左右三叉作为叉41进入。 */
    fork40_blind_mode = (subject_id == 2U &&
        route_direction == ROUTE_DIRECTION_FORWARD &&
        !fork_correction_enable &&
        fork_num == FORK_PRELIM_FORWARD_BLIND_INDEX);
    fork40_left_right_detected = (fork_dbg_src_L == 2 &&
        fork_dbg_src_R == 2);

    if (car_state == normal)
    {
        if (fork_correction_enable &&
            route_progress_force_enter_requested() &&
            subject_id != 1U)
        {
            state_enter_current_fork(FORK_CORR_SOURCE_FORCED);
        }
        else if (vision_entry_condition &&
                 (fork40_blind_mode
                      ? fork40_left_right_detected
                      : (!exit_pair_required || exit_pair_matches)))
        {
            int can_enter = 1;
            count++;
            if (fork_correction_enable)
            {
                can_enter = route_progress_vision_entry_allowed();
            }

            if (count >= FORK_ENTRY_DEBOUNCE)
            {
                if (can_enter && subject_id != 1U &&
                    fork_reentry_block_frames == 0)
                {
                    if (fork40_blind_mode)
                    {
                        fork_num = 41;
                    }
                    state_enter_current_fork(FORK_CORR_SOURCE_VISION);
                }
                count = 0;
            }
        }
        else
        {
            count = 0;
        }

        if (fork_reentry_block_frames > 0)
        {
            fork_reentry_block_frames--;
        }
    }
    else if (fork_correction_enable &&
             route_progress_force_exit_requested())
    {
        state_finish_current_fork(FORK_CORR_SOURCE_FORCED);
    }
    else
    {
        switch (car_state)
        {
            case fork_straight:
            {
                if (state_straight_fork_distance_reached())
                {
                    state_finish_current_fork(
                        FORK_CORR_SOURCE_NORMAL_EXIT);
                    break;
                }

                if (fork_correction_enable &&
                    !route_progress_normal_exit_allowed())
                {
                    state_clear_exit_evidence();
                    break;
                }

                if (trend.slope < 100 &&
                    (detected_exit_pair == FORK_EXIT_PAIR_INVALID ||
                     detected_exit_pair != fork_entry_exit_pair))
                {
                    straight_count++;
                    if (straight_count > 2)
                    {
                        state_finish_current_fork(
                            FORK_CORR_SOURCE_NORMAL_EXIT);
                    }
                }
                else
                {
                    straight_count = 0;
                }
                break;
            }

            case fork_turn:
            {
                fork_turn_frames++;
                angle_difference = yaw_angle_difference(
                    get_yaw(),
                    current_angle);

                if (fork_correction_enable &&
                    !route_progress_normal_exit_allowed())
                {
                    state_clear_exit_evidence();
                    break;
                }

                if (angle_difference >= FORK_EXIT_PREPARE_ANGLE &&
                    element_count < 1)
                {
                    fork_turn_lost_count++;
                }
                else
                {
                    fork_turn_lost_count = 0;
                }

                if (fork_turn_lost_count >=
                    FORK_EXIT_LOST_CONFIRM_FRAMES)
                {
                    state_finish_current_fork(
                        FORK_CORR_SOURCE_NORMAL_EXIT);
                    break;
                }

                break;
            }

            case fork_exit_pending:
            {
                state_finish_current_fork(
                    FORK_CORR_SOURCE_NORMAL_EXIT);
                break;
            }

            case normal:
            default:
                break;
        }
    }

    switch (line_state)
    {
        case none:
        {
            if (car_state == normal && check_sharp_turn_state(angle_detect_lookahead) == 1)
            {
                current_angle = get_yaw();
                turn_end_damping_active = 0U;
                line_state = angle;
                if (run_features_map_enabled())
                {
                    INS_Relocalize_At_Flag(
                        (int)trajectory_encode_angle(angle_num));
                }
            }
            break;
        }

        case angle:
        {
            if (car_state == normal)
            {
                end_angle = get_yaw();
                angle_difference = yaw_angle_difference(
                    end_angle,
                    current_angle);
                if (angle_difference > 60.0f)
                {
                    line_state = none;
                    turn_end_damping_active = 0U;
                    angle_num++;
                }
            }
            else
            {
                line_state = none;
                turn_end_damping_active = 0U;
            }
            break;
        }
    }
}

void show_state(void)
{
    switch (car_state)
    {
        case normal:
            ips114_show_string(0, 100, "normal");
            break;
        case fork_turn:
            ips114_show_string(0, 100, "fork_turn");
            break;
        case fork_straight:
            ips114_show_string(0, 100, "fork_straight");
            break;
        case fork_exit_pending:
            ips114_show_string(0, 100, "fork_pending");
            break;
    }
}
