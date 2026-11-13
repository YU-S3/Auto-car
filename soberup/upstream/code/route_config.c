#include "route_config.h"

#include <stddef.h>

static const int fork_type_prelim_forward[FORK_CONFIG_CAPACITY] = {
    0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0,
    0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_type_prelim_reverse[FORK_CONFIG_CAPACITY] = {
    0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_type_final_forward[FORK_CONFIG_CAPACITY] = {
    1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1,
    0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_type_final_reverse[FORK_CONFIG_CAPACITY] = {
    0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1,
    0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_seq_prelim_forward[FORK_CONFIG_CAPACITY] = {
    0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 2, 0, 0, 2, 0,
    1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1,
    1, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_seq_prelim_reverse[FORK_CONFIG_CAPACITY] = {
    1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_seq_final_forward[FORK_CONFIG_CAPACITY] = {
    1, 0, 1, 0, 2, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 0, 0, 0, 0, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_seq_final_reverse[FORK_CONFIG_CAPACITY] = {
    1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1,
    0, 1, 1, 0, 0, 1, 1, 2, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* 预赛正向穿出边界对：1=左上 2=上右 3=左右；叉40盲区跳过、叉41=左右，实车标定。 */
static const uint8 fork_exit_pair_prelim_forward[FORK_CONFIG_CAPACITY] = {
    2, 2, 3, 2, 2, 2, 1, 3, 2, 3, 2, 3, 2, 1, 2, 3, 3, 3, 3, 2,
    1, 3, 2, 2, 3, 1, 2, 3, 2, 3, 1, 3, 2, 2, 1, 3, 3, 2, 3, 3,
    2, 3, 3, 1, 3, 1, 3, 1, 2, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fork_exit_pair_prelim_reverse[FORK_CONFIG_CAPACITY] = {
    1, 1, 1, 1, 1, 3, 1, 1, 1, 1, 1, 3, 3, 2, 1, 3, 2, 2, 3, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fork_exit_pair_final_forward[FORK_CONFIG_CAPACITY] = {
    2, 1, 1, 3, 3, 3, 2, 3, 3, 3, 3, 1, 1, 2, 3, 3, 1, 1, 1, 2,
    3, 3, 3, 3, 3, 2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 2, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fork_exit_pair_final_reverse[FORK_CONFIG_CAPACITY] = {
    1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 3, 1, 1, 1, 2, 3, 3, 3, 3, 3,
    2, 2, 3, 3, 2, 2, 3, 3, 1, 3, 1, 3, 1, 2, 3, 3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fill_mode_prelim_forward[FORK_CONFIG_CAPACITY] = {
    0, 2, 2, 2, 0, 0, 2, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0,
    0, 2, 2, 0, 2, 0, 2, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fill_mode_prelim_reverse[FORK_CONFIG_CAPACITY] = {
    2, 0, 0, 2, 0, 0, 2, 0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 0, 0, 0,
    2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fill_mode_final_forward[FORK_CONFIG_CAPACITY] = {
    2, 0, 0, 0, 0, 0, 2, 0, 0, 2, 0, 0, 2, 0, 0, 0, 2, 1, 0, 1,
    0, 2, 2, 2, 0, 2, 2, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8 fill_mode_final_reverse[FORK_CONFIG_CAPACITY] = {
    2, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 0, 2, 0, 0, 2, 0, 0, 2, 0,
    0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* 角度观测器参考航向（度，网格赛道为 90 的倍数）：入叉时实测 yaw 与该值
   偏差超过阈值即可能走错方向。其余三条路线暂未提供数据，全置 0 预留。 */
static const int fork_entry_angle_prelim_forward[FORK_CONFIG_CAPACITY] = {
    0, 90, -90, 90, 90, 90, -180, -180, -90, -90, 0, 0, 90, -180, 90, -180, -180, 90, 0, 0,
    0, 0, 90, 90, 0, -90, -180, -180, -180, -180, 90, 0, -90, -90, 0, -90, -180, -90, 0, -90,
    -180, -90, -180, 90, 90, 0, -90, -90, -180, -180, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_entry_angle_prelim_reverse[FORK_CONFIG_CAPACITY] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_entry_angle_final_forward[FORK_CONFIG_CAPACITY] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const int fork_entry_angle_final_reverse[FORK_CONFIG_CAPACITY] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

#define ROUTE_ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define ROUTE_SIZE_CHECK(name) \
    typedef char name##_size_check[ \
        (ROUTE_ARRAY_COUNT(name) == FORK_CONFIG_CAPACITY) ? 1 : -1]

ROUTE_SIZE_CHECK(fork_type_prelim_forward);
ROUTE_SIZE_CHECK(fork_type_prelim_reverse);
ROUTE_SIZE_CHECK(fork_type_final_forward);
ROUTE_SIZE_CHECK(fork_type_final_reverse);
ROUTE_SIZE_CHECK(fork_seq_prelim_forward);
ROUTE_SIZE_CHECK(fork_seq_prelim_reverse);
ROUTE_SIZE_CHECK(fork_seq_final_forward);
ROUTE_SIZE_CHECK(fork_seq_final_reverse);
ROUTE_SIZE_CHECK(fork_exit_pair_prelim_forward);
ROUTE_SIZE_CHECK(fork_exit_pair_prelim_reverse);
ROUTE_SIZE_CHECK(fork_exit_pair_final_forward);
ROUTE_SIZE_CHECK(fork_exit_pair_final_reverse);
ROUTE_SIZE_CHECK(fill_mode_prelim_forward);
ROUTE_SIZE_CHECK(fill_mode_prelim_reverse);
ROUTE_SIZE_CHECK(fill_mode_final_forward);
ROUTE_SIZE_CHECK(fill_mode_final_reverse);
ROUTE_SIZE_CHECK(fork_entry_angle_prelim_forward);
ROUTE_SIZE_CHECK(fork_entry_angle_prelim_reverse);
ROUTE_SIZE_CHECK(fork_entry_angle_final_forward);
ROUTE_SIZE_CHECK(fork_entry_angle_final_reverse);

static const int *route_select_int_table(
    uint8 subject_id,
    uint8 route_direction,
    const int *prelim_forward,
    const int *prelim_reverse,
    const int *final_forward,
    const int *final_reverse)
{
    if (subject_id == 2U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? prelim_forward : prelim_reverse;
    }
    if (subject_id == 3U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? final_forward : final_reverse;
    }
    return NULL;
}

static const uint8 *route_select_uint8_table(
    uint8 subject_id,
    uint8 route_direction,
    const uint8 *prelim_forward,
    const uint8 *prelim_reverse,
    const uint8 *final_forward,
    const uint8 *final_reverse)
{
    if (subject_id == 2U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? prelim_forward : prelim_reverse;
    }
    if (subject_id == 3U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? final_forward : final_reverse;
    }
    return NULL;
}

int route_fork_index_valid(int fork_index)
{
    return fork_index >= 0 && fork_index < FORK_CONFIG_CAPACITY;
}

uint8 route_is_encoder_calibration(uint8 subject_id)
{
    return (uint8)(subject_id == ROUTE_SUBJECT_ENCODER_CAL);
}

int route_get_actual_fork_count(uint8 subject_id, uint8 route_direction)
{
    (void)route_direction;
    if (route_is_encoder_calibration(subject_id))
    {
        return 2;
    }
    if (subject_id == 2U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? ROUTE_ACTUAL_FORK_PRELIM_FORWARD
            : ROUTE_ACTUAL_FORK_PRELIM_REVERSE;
    }
    if (subject_id == 3U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? ROUTE_ACTUAL_FORK_FINAL_FORWARD
            : ROUTE_ACTUAL_FORK_FINAL_REVERSE;
    }
    return 0;
}

int route_fork_entry_angle_valid(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index)
{
    return route_fork_index_valid(fork_index) &&
        fork_index < route_get_actual_fork_count(
            subject_id, route_direction);
}

int route_get_fork_entry_angle_deg(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index)
{
    const int *table;

    if (route_is_encoder_calibration(subject_id))
    {
        return 0;
    }

    if (!route_fork_entry_angle_valid(
            subject_id, route_direction, fork_index))
    {
        return 0;
    }
    table = route_select_int_table(
        subject_id, route_direction,
        fork_entry_angle_prelim_forward, fork_entry_angle_prelim_reverse,
        fork_entry_angle_final_forward, fork_entry_angle_final_reverse);
    return table != NULL ? table[fork_index] : 0;
}

int route_get_fork_type(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index)
{
    const int *table;

    if (!route_fork_index_valid(fork_index))
    {
        return FORK_TYPE_TURN;
    }
    table = route_select_int_table(
        subject_id, route_direction,
        fork_type_prelim_forward, fork_type_prelim_reverse,
        fork_type_final_forward, fork_type_final_reverse);
    return table != NULL ? table[fork_index] : FORK_TYPE_TURN;
}

int route_get_fork_seq(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index)
{
    const int *table;

    if (!route_fork_index_valid(fork_index))
    {
        return FORK_SEQ_LEFT;
    }
    table = route_select_int_table(
        subject_id, route_direction,
        fork_seq_prelim_forward, fork_seq_prelim_reverse,
        fork_seq_final_forward, fork_seq_final_reverse);
    return table != NULL ? table[fork_index] : FORK_SEQ_LEFT;
}

uint8 route_classify_fork_exit_pair(int left_boundary, int right_boundary)
{
    if (left_boundary == FORK_EXIT_BOUNDARY_LEFT &&
        right_boundary == FORK_EXIT_BOUNDARY_TOP)
    {
        return FORK_EXIT_PAIR_LEFT_TOP;
    }
    if (left_boundary == FORK_EXIT_BOUNDARY_TOP &&
        right_boundary == FORK_EXIT_BOUNDARY_RIGHT)
    {
        return FORK_EXIT_PAIR_TOP_RIGHT;
    }
    if (left_boundary == FORK_EXIT_BOUNDARY_LEFT &&
        right_boundary == FORK_EXIT_BOUNDARY_RIGHT)
    {
        return FORK_EXIT_PAIR_LEFT_RIGHT;
    }
    return FORK_EXIT_PAIR_INVALID;
}

uint8 route_get_fork_exit_pair(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index)
{
    const uint8 *table;

    if (!route_fork_index_valid(fork_index))
    {
        return FORK_EXIT_PAIR_INVALID;
    }
    table = route_select_uint8_table(
        subject_id, route_direction,
        fork_exit_pair_prelim_forward, fork_exit_pair_prelim_reverse,
        fork_exit_pair_final_forward, fork_exit_pair_final_reverse);
    return table != NULL ? table[fork_index] : FORK_EXIT_PAIR_INVALID;
}

int route_fork_exit_pair_matches(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index,
    int left_boundary,
    int right_boundary)
{
    uint8 expected = route_get_fork_exit_pair(
        subject_id, route_direction, fork_index);
    return expected != FORK_EXIT_PAIR_INVALID &&
        expected == route_classify_fork_exit_pair(
            left_boundary,
            right_boundary);
}

uint8 route_get_fill_mode(
    uint8 subject_id,
    uint8 route_direction,
    int fork_index)
{
    const uint8 *table;

    if (!route_fork_index_valid(fork_index))
    {
        return FILL_NONE;
    }
    table = route_select_uint8_table(
        subject_id, route_direction,
        fill_mode_prelim_forward, fill_mode_prelim_reverse,
        fill_mode_final_forward, fill_mode_final_reverse);
    return table != NULL ? table[fork_index] : FILL_NONE;
}

int route_get_stop_fork_num(uint8 subject_id, uint8 route_direction)
{
    if (subject_id == 2U)
    {
        return route_direction == ROUTE_DIRECTION_FORWARD
            ? ROUTE_STOP_FORK_PRELIM_FORWARD
            : ROUTE_STOP_FORK_PRELIM_REVERSE;
    }
    return route_direction == ROUTE_DIRECTION_FORWARD
        ? ROUTE_STOP_FORK_FINAL_FORWARD
        : ROUTE_STOP_FORK_FINAL_REVERSE;
}
