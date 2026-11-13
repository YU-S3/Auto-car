#include "inertial_navigation.h"

#include "line.h"
#include "map_event.h"
#include "route_config.h"
#include "route_progress.h"
#include "run_features.h"
#include "state_control.h"
#include "trajectory_policy.h"
#include "zf_driver_flash.h"

#include <math.h>
#include <string.h>

#define TRAJECTORY_ERROR_FILTER_ALPHA 0.7f
#define TRAJECTORY_LOW_SPEED_BEFORE_POINTS 30
#define TRAJECTORY_LOW_SPEED_AFTER_POINTS  10
#define TRAJECTORY_DRS_FLAG_BYTES     ((MAX_PATH_POINTS + 7U) / 8U)

#define TRAJECTORY_MAGIC             0x54524A32U
#define TRAJECTORY_FORMAT_VERSION    2U
#define TRAJECTORY_META_PAGE         0U
#define TRAJECTORY_DATA_FIRST_PAGE   5U
#define TRAJECTORY_DATA_LAST_PAGE    10U

typedef struct
{
    uint32_t magic;
    uint32_t format_version;
    uint32_t record_size;
    uint32_t point_count;
    uint32_t total_mileage_ticks;
    uint32_t planned_min_speed;
    uint32_t planned_max_speed;
    uint32_t decel_step;
    uint32_t accel_step;
    uint32_t payload_crc32;
} TrajectoryMeta;

typedef char trajectory_meta_word_aligned[
    (sizeof(TrajectoryMeta) % sizeof(uint32_t) == 0U) ? 1 : -1];

extern int angle_num;
extern int fork_num;
extern uint8 subject_id;
extern uint8 route_direction;

Car_State_t car_pos;

#pragma section all "cpu0_dsram"
INS_Pos_t Path_Memory[MAX_PATH_POINTS];
static uint8_t trajectory_drs_flags[TRAJECTORY_DRS_FLAG_BYTES];
static uint32_t trajectory_flash_buffer[EEPROM_PAGE_LENGTH];
#pragma section all restore

int Total_Path_Points = 0;
int Current_Path_Index = 0;
int min_target_speed = 1000;
int max_target_speed = 1600;
int DECEL_STEP = TRAJECTORY_DECEL_DEFAULT;
int ACCEL_STEP = TRAJECTORY_ACCEL_DEFAULT;

uint8_t trajectory_map_valid = 0U;
uint8_t path_record_full = 0U;
TrajectoryResult trajectory_last_result = TRAJECTORY_EMPTY;
volatile uint32_t ins_relocalize_attempt_count = 0U;
volatile uint32_t ins_relocalize_success_count = 0U;
volatile int32_t ins_relocalize_last_correction_ticks = 0;

static uint32_t next_record_mileage = RECORD_INTERVAL_TICKS;
static int8_t interval_peak_error = 0;
static uint8_t interval_element_code = TRAJECTORY_ELEMENT_NONE;

static void trajectory_set_start_speed_to_max(void)
{
    if (Total_Path_Points > 0)
    {
        Path_Memory[0].target_speed = (uint16_t)max_target_speed;
    }
}

static void trajectory_set_drs_flag(int index)
{
    trajectory_drs_flags[(uint32_t)index / 8U] |=
        (uint8_t)(1U << ((uint32_t)index % 8U));
}

static void trajectory_rebuild_drs_flags(void)
{
    int platform_start = -1;
    int i;

    memset(trajectory_drs_flags, 0, sizeof(trajectory_drs_flags));

    if (Total_Path_Points <= 0 || max_target_speed <= 0)
    {
        return;
    }

    for (i = 0; i <= Total_Path_Points; i++)
    {
        uint8_t is_max_speed_point = (uint8_t)(
            i < Total_Path_Points &&
            (int)Path_Memory[i].target_speed == max_target_speed);

        if (is_max_speed_point)
        {
            if (platform_start < 0)
            {
                platform_start = i;
            }
        }
        else if (platform_start >= 0)
        {
            int platform_end = i - 1;
            uint32_t platform_length =
                Path_Memory[platform_end].mileage_ticks -
                Path_Memory[platform_start].mileage_ticks;

            if (platform_length >= TRAJECTORY_DRS_MIN_TICKS)
            {
                int j;
                uint32_t drs_end_mileage =
                    Path_Memory[platform_end].mileage_ticks -
                    TRAJECTORY_DRS_RECOVERY_TICKS;

                for (j = platform_start;
                     j <= platform_end &&
                     Path_Memory[j].mileage_ticks <= drs_end_mileage;
                     j++)
                {
                    trajectory_set_drs_flag(j);
                }
            }
            platform_start = -1;
        }
    }
}

static int trajectory_clamp_int(int value, int lower, int upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static int trajectory_round_float(float value)
{
    return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static int trajectory_abs_int(int value)
{
    return value < 0 ? -value : value;
}

static uint8_t trajectory_plan_params_valid(
    int min_speed,
    int max_speed,
    int decel_step,
    int accel_step)
{
    return (uint8_t)(
        min_speed >= 0 &&
        min_speed <= max_speed &&
        max_speed <= TRAJECTORY_TARGET_SPEED_MAX &&
        decel_step >= TRAJECTORY_STEP_MIN &&
        decel_step <= TRAJECTORY_STEP_MAX &&
        accel_step >= TRAJECTORY_STEP_MIN &&
        accel_step <= TRAJECTORY_STEP_MAX);
}

static uint8_t trajectory_current_element_code(void)
{
    if (car_state != normal)
    {
        return trajectory_encode_fork(fork_num);
    }
    if (line_state != none)
    {
        return trajectory_encode_angle(angle_num);
    }
    return TRAJECTORY_ELEMENT_NONE;
}

static void trajectory_reset_interval_recorder(void)
{
    next_record_mileage = RECORD_INTERVAL_TICKS;
    interval_peak_error = 0;
    interval_element_code = TRAJECTORY_ELEMENT_NONE;
}

static uint8_t trajectory_element_requires_low_speed(uint8_t element_code)
{
    TrajectoryElementType element_type =
        trajectory_element_type(element_code);

    if (element_type == TRAJECTORY_ELEMENT_TYPE_ANGLE)
    {
        return 1U;
    }
    if (element_type == TRAJECTORY_ELEMENT_TYPE_FORK)
    {
        int fnum = trajectory_element_id(element_code);
        if (!route_fork_index_valid(fnum))
        {
            return 1U;
        }
        return (uint8_t)(
            route_get_fork_type(
                subject_id, route_direction, fnum) == FORK_TYPE_TURN);
    }
    return 0U;
}

static uint32_t trajectory_crc32_update(
    uint32_t crc,
    const void *data,
    uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t i;

    for (i = 0U; i < length; i++)
    {
        uint8_t bit;
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint32_t trajectory_crc32(const void *data, uint32_t length)
{
    return trajectory_crc32_update(0xFFFFFFFFU, data, length) ^
        0xFFFFFFFFU;
}

// Flash 驱动用全局标志记录“刚擦过一页”，下一次写入会跳过自动擦除。
// 因此擦除元数据页后必须立即向同一页写零，消费该标志；否则随后写
// 数据页时可能错误地跳过该数据页的擦除。
static void trajectory_invalidate_flash_map(void)
{
    const uint32_t empty_marker = 0U;

    flash_erase_page(0, TRAJECTORY_META_PAGE);
    flash_write_page(
        0,
        TRAJECTORY_META_PAGE,
        (const uint32 *)&empty_marker,
        1U);
}

static TrajectoryResult trajectory_publish_failure(TrajectoryResult result)
{
    Total_Path_Points = 0;
    Current_Path_Index = 0;
    trajectory_map_valid = 0U;
    path_record_full = 0U;
    trajectory_last_result = result;
    memset(trajectory_drs_flags, 0, sizeof(trajectory_drs_flags));
    route_progress_clear();
    return result;
}

static uint8_t trajectory_path_data_valid(
    const INS_Pos_t *path,
    uint32_t point_count,
    uint32_t total_mileage_ticks)
{
    uint32_t i;

    if (point_count == 0U || point_count > MAX_PATH_POINTS)
    {
        return 0U;
    }

    for (i = 0U; i < point_count; i++)
    {
        if (path[i].target_speed > TRAJECTORY_TARGET_SPEED_MAX ||
            path[i].peak_error < -TRAJECTORY_PEAK_ERROR_MAX ||
            path[i].peak_error > TRAJECTORY_PEAK_ERROR_MAX)
        {
            return 0U;
        }
        if (i > 0U && path[i].mileage_ticks <= path[i - 1U].mileage_ticks)
        {
            return 0U;
        }
    }

    return (uint8_t)(
        path[point_count - 1U].mileage_ticks == total_mileage_ticks);
}

static void trajectory_flush_final_interval(void)
{
    uint32_t current_mileage;

    if (!record_enable || car_pos.mileage_ticks <= 0.0f ||
        Total_Path_Points >= MAX_PATH_POINTS)
    {
        return;
    }

    current_mileage = (uint32_t)car_pos.mileage_ticks;
    if (current_mileage == 0U)
    {
        return;
    }
    if (Total_Path_Points > 0 &&
        current_mileage <= Path_Memory[Total_Path_Points - 1].mileage_ticks)
    {
        return;
    }

    Path_Memory[Total_Path_Points].mileage_ticks = current_mileage;
    Path_Memory[Total_Path_Points].target_speed = 0U;
    Path_Memory[Total_Path_Points].element_code = interval_element_code;
    Path_Memory[Total_Path_Points].peak_error = interval_peak_error;
    Total_Path_Points++;

    if (Total_Path_Points >= MAX_PATH_POINTS)
    {
        path_record_full = 1U;
    }

    interval_peak_error = 0;
    interval_element_code = TRAJECTORY_ELEMENT_NONE;
    while (car_pos.mileage_ticks >= (float)next_record_mileage)
    {
        next_record_mileage += RECORD_INTERVAL_TICKS;
    }
}

static uint8_t trajectory_verify_flash_write(
    const TrajectoryMeta *expected_meta)
{
    TrajectoryMeta stored_meta;
    uint32_t total_words;
    uint32_t words_read = 0U;
    uint32_t current_page = TRAJECTORY_DATA_FIRST_PAGE;
    uint32_t crc = 0xFFFFFFFFU;

    if (flash_check(0, TRAJECTORY_META_PAGE) != 1)
    {
        return 0U;
    }

    flash_read_page(
        0,
        TRAJECTORY_META_PAGE,
        (uint32 *)&stored_meta,
        (uint16)(sizeof(stored_meta) / sizeof(uint32_t)));
    if (memcmp(&stored_meta, expected_meta, sizeof(stored_meta)) != 0)
    {
        return 0U;
    }

    total_words = expected_meta->point_count *
        (uint32_t)(sizeof(INS_Pos_t) / sizeof(uint32_t));
    while (words_read < total_words &&
           current_page <= TRAJECTORY_DATA_LAST_PAGE)
    {
        uint32_t words_to_read = total_words - words_read;
        uint32_t byte_count;

        if (words_to_read > EEPROM_PAGE_LENGTH)
        {
            words_to_read = EEPROM_PAGE_LENGTH;
        }

        flash_read_page(
            0,
            current_page,
            trajectory_flash_buffer,
            (uint16)words_to_read);
        byte_count = words_to_read * (uint32_t)sizeof(uint32_t);
        if (memcmp(
                trajectory_flash_buffer,
                (const uint8_t *)Path_Memory +
                    words_read * (uint32_t)sizeof(uint32_t),
                byte_count) != 0)
        {
            return 0U;
        }

        crc = trajectory_crc32_update(
            crc,
            trajectory_flash_buffer,
            byte_count);
        words_read += words_to_read;
        current_page++;
    }

    return (uint8_t)(
        words_read == total_words &&
        (crc ^ 0xFFFFFFFFU) == expected_meta->payload_crc32);
}

void INS_Init(void)
{
    car_pos.mileage_ticks = 0.0f;
    car_pos.yaw = 0.0f;
    car_pos.filtered_error = 0.0f;
    car_pos.error = 0;
    car_pos.element_code = TRAJECTORY_ELEMENT_NONE;

    Current_Path_Index = 0;
    ins_relocalize_attempt_count = 0U;
    ins_relocalize_success_count = 0U;
    ins_relocalize_last_correction_ticks = 0;
    trajectory_reset_interval_recorder();

    if (record_enable)
    {
        Total_Path_Points = 0;
        trajectory_map_valid = 0U;
        path_record_full = 0U;
        trajectory_last_result = TRAJECTORY_EMPTY;
        memset(trajectory_drs_flags, 0, sizeof(trajectory_drs_flags));
        route_progress_clear();
    }
}

void INS_Update(int enc_L, int enc_R, float imu_yaw)
{
    float delta_ticks = (enc_L + enc_R) / 2.0f;
    int filtered_error;

    car_pos.yaw = imu_yaw;
    car_pos.mileage_ticks += fabsf(delta_ticks);
    car_pos.error = return_error();
    car_pos.filtered_error =
        TRAJECTORY_ERROR_FILTER_ALPHA * (float)car_pos.error +
        (1.0f - TRAJECTORY_ERROR_FILTER_ALPHA) * car_pos.filtered_error;
    car_pos.element_code = trajectory_current_element_code();

    if (!record_enable)
    {
        return;
    }

    filtered_error = trajectory_round_float(car_pos.filtered_error);
    filtered_error = trajectory_clamp_int(
        filtered_error,
        -TRAJECTORY_PEAK_ERROR_MAX,
        TRAJECTORY_PEAK_ERROR_MAX);

    if (trajectory_abs_int(filtered_error) >
        trajectory_abs_int((int)interval_peak_error))
    {
        interval_peak_error = (int8_t)filtered_error;
    }

    if (interval_element_code == TRAJECTORY_ELEMENT_NONE &&
        car_pos.element_code != TRAJECTORY_ELEMENT_NONE)
    {
        interval_element_code = car_pos.element_code;
    }
}

void INS_Clear_Map(void)
{
    Total_Path_Points = 0;
    Current_Path_Index = 0;
    trajectory_map_valid = 0U;
    path_record_full = 0U;
    trajectory_last_result = TRAJECTORY_EMPTY;
    memset(trajectory_drs_flags, 0, sizeof(trajectory_drs_flags));
    trajectory_reset_interval_recorder();
    route_progress_clear();
}

void INS_Record_Trajectory(void)
{
    if (!record_enable)
    {
        return;
    }
    if (car_pos.mileage_ticks < (float)next_record_mileage)
    {
        return;
    }
    if (Total_Path_Points >= MAX_PATH_POINTS)
    {
        path_record_full = 1U;
        return;
    }

    Path_Memory[Total_Path_Points].mileage_ticks =
        (uint32_t)car_pos.mileage_ticks;
    Path_Memory[Total_Path_Points].target_speed = 0U;
    Path_Memory[Total_Path_Points].element_code = interval_element_code;
    Path_Memory[Total_Path_Points].peak_error = interval_peak_error;
    Total_Path_Points++;

    if (Total_Path_Points >= MAX_PATH_POINTS)
    {
        path_record_full = 1U;
    }

    interval_peak_error = 0;
    interval_element_code = TRAJECTORY_ELEMENT_NONE;
    do
    {
        next_record_mileage += RECORD_INTERVAL_TICKS;
    }
    while (car_pos.mileage_ticks >= (float)next_record_mileage);
}

uint8_t INS_Generate_Velocity_Table(int min_speed, int max_speed)
{
    int i;
    int element_start_index = -1;

    if (Total_Path_Points <= 0 || Total_Path_Points > MAX_PATH_POINTS ||
        !trajectory_plan_params_valid(
            min_speed,
            max_speed,
            DECEL_STEP,
            ACCEL_STEP))
    {
        return 0U;
    }

    min_target_speed = min_speed;
    max_target_speed = max_speed;

    for (i = 0; i < Total_Path_Points; i++)
    {
        int element_speed_cap =
            trajectory_element_requires_low_speed(
                Path_Memory[i].element_code)
            ? min_speed
            : max_speed;
        int error_speed_cap = trajectory_error_speed_cap(
            (int)Path_Memory[i].peak_error,
            min_speed,
            max_speed);
        int point_speed = element_speed_cap < error_speed_cap
            ? element_speed_cap
            : error_speed_cap;

        Path_Memory[i].target_speed = (uint16_t)trajectory_clamp_int(
            point_speed,
            0,
            TRAJECTORY_TARGET_SPEED_MAX);
    }

    for (i = 0; i <= Total_Path_Points; i++)
    {
        uint8_t is_low_speed_element = 0U;

        if (i < Total_Path_Points)
        {
            is_low_speed_element = trajectory_element_requires_low_speed(
                Path_Memory[i].element_code);
        }

        if (is_low_speed_element)
        {
            if (element_start_index < 0)
            {
                element_start_index = i;
            }
        }
        else if (element_start_index >= 0)
        {
            int element_end_index = i - 1;
            int middle_index =
                element_start_index +
                (element_end_index - element_start_index + 1) / 2;
            int platform_start =
                middle_index - TRAJECTORY_LOW_SPEED_BEFORE_POINTS;
            int platform_end =
                middle_index + TRAJECTORY_LOW_SPEED_AFTER_POINTS;
            int j;

            if (platform_start < 0)
            {
                platform_start = 0;
            }
            if (platform_end >= Total_Path_Points)
            {
                platform_end = Total_Path_Points - 1;
            }

            for (j = platform_start; j <= platform_end; j++)
            {
                Path_Memory[j].target_speed = (uint16_t)min_speed;
            }
            element_start_index = -1;
        }
    }

    for (i = Total_Path_Points - 2; i >= 0; i--)
    {
        int safe_braking_speed =
            (int)Path_Memory[i + 1].target_speed + DECEL_STEP;
        if ((int)Path_Memory[i].target_speed > safe_braking_speed)
        {
            Path_Memory[i].target_speed = (uint16_t)safe_braking_speed;
        }
    }

    for (i = 1; i < Total_Path_Points; i++)
    {
        int safe_accel_speed =
            (int)Path_Memory[i - 1].target_speed + ACCEL_STEP;
        if ((int)Path_Memory[i].target_speed > safe_accel_speed)
        {
            Path_Memory[i].target_speed = (uint16_t)safe_accel_speed;
        }
    }

    trajectory_set_start_speed_to_max();
    trajectory_rebuild_drs_flags();
    return 1U;
}

void INS_Sync_Tracking_Index(void)
{
    if (Total_Path_Points <= 0)
    {
        return;
    }

    while (Current_Path_Index < Total_Path_Points - 1 &&
           car_pos.mileage_ticks >=
               (float)Path_Memory[Current_Path_Index + 1].mileage_ticks)
    {
        Current_Path_Index++;
    }
}

int32_t INS_Get_Target_Speed(void)
{
    int table_speed;
    int live_error_cap;
    int filtered_error;

    if (!speed_decision_enable || !trajectory_map_valid ||
        Total_Path_Points <= 0)
    {
        return min_target_speed;
    }

    INS_Sync_Tracking_Index();
    if (Current_Path_Index >= Total_Path_Points - 1)
    {
        return min_target_speed;
    }

    table_speed = (int)Path_Memory[Current_Path_Index].target_speed;
    filtered_error = trajectory_round_float(car_pos.filtered_error);
    live_error_cap = (int)trajectory_error_speed_cap(
        filtered_error,
        min_target_speed,
        max_target_speed);

    return table_speed < live_error_cap ? table_speed : live_error_cap;
}

uint8_t INS_Is_DRS_Eligible(void)
{
    uint32_t index;

    if (!speed_decision_enable || !trajectory_map_valid ||
        Total_Path_Points <= 0)
    {
        return 0U;
    }

    INS_Sync_Tracking_Index();
    if (Current_Path_Index < 0 || Current_Path_Index >= Total_Path_Points)
    {
        return 0U;
    }

    index = (uint32_t)Current_Path_Index;
    return (uint8_t)(
        (trajectory_drs_flags[index / 8U] &
         (uint8_t)(1U << (index % 8U))) != 0U);
}

INS_Pos_t INS_Get_Lookahead_Point(float lookahead_dist_ticks)
{
    int lookahead_index;
    float target_mileage;

    if (Total_Path_Points <= 0)
    {
        INS_Pos_t empty_point = {0};
        return empty_point;
    }

    INS_Sync_Tracking_Index();
    target_mileage = car_pos.mileage_ticks + lookahead_dist_ticks;
    lookahead_index = Current_Path_Index;

    while (lookahead_index < Total_Path_Points - 1 &&
           (float)Path_Memory[lookahead_index].mileage_ticks < target_mileage)
    {
        lookahead_index++;
    }
    return Path_Memory[lookahead_index];
}

void INS_Relocalize_At_Flag(int target_flag)
{
    int search_start;
    int search_end;
    int i;

    if (Total_Path_Points <= 0 || !trajectory_map_valid ||
        !run_features_map_enabled() || target_flag <= 0)
    {
        return;
    }

    ins_relocalize_attempt_count++;

    search_start = Current_Path_Index - 60;
    if (search_start < 0)
    {
        search_start = 0;
    }
    search_end = Current_Path_Index + 60;
    if (search_end >= Total_Path_Points)
    {
        search_end = Total_Path_Points - 1;
    }

    for (i = search_start; i <= search_end; i++)
    {
        if ((int)Path_Memory[i].element_code == target_flag)
        {
            float drift =
                (float)Path_Memory[i].mileage_ticks - car_pos.mileage_ticks;
            if (fabsf(drift) < 5000.0f)
            {
                car_pos.mileage_ticks =
                    (float)Path_Memory[i].mileage_ticks;
                Current_Path_Index = i;
                ins_relocalize_success_count++;
                ins_relocalize_last_correction_ticks = (int32_t)drift;
            }
            break;
        }
    }
}

TrajectoryResult Load_Trajectory_From_Flash(void)
{
    TrajectoryMeta meta;
    uint32_t total_words;
    uint32_t words_read = 0U;
    uint32_t current_page = TRAJECTORY_DATA_FIRST_PAGE;
    uint32_t actual_crc;

    trajectory_map_valid = 0U;
    Total_Path_Points = 0;
    Current_Path_Index = 0;
    route_progress_clear();

    if (flash_check(0, TRAJECTORY_META_PAGE) != 1)
    {
        return trajectory_publish_failure(TRAJECTORY_EMPTY);
    }

    flash_read_page(
        0,
        TRAJECTORY_META_PAGE,
        (uint32 *)&meta,
        (uint16)(sizeof(meta) / sizeof(uint32_t)));

    if (meta.magic != TRAJECTORY_MAGIC)
    {
        return trajectory_publish_failure(TRAJECTORY_OLD_FORMAT);
    }
    if (meta.format_version != TRAJECTORY_FORMAT_VERSION ||
        meta.record_size != sizeof(INS_Pos_t) ||
        meta.point_count == 0U ||
        meta.point_count > MAX_PATH_POINTS ||
        !trajectory_plan_params_valid(
            (int)meta.planned_min_speed,
            (int)meta.planned_max_speed,
            (int)meta.decel_step,
            (int)meta.accel_step))
    {
        return trajectory_publish_failure(TRAJECTORY_META_INVALID);
    }

    total_words = meta.point_count *
        (uint32_t)(sizeof(INS_Pos_t) / sizeof(uint32_t));
    while (words_read < total_words &&
           current_page <= TRAJECTORY_DATA_LAST_PAGE)
    {
        uint32_t words_to_read = total_words - words_read;
        uint32_t byte_count;

        if (words_to_read > EEPROM_PAGE_LENGTH)
        {
            words_to_read = EEPROM_PAGE_LENGTH;
        }
        flash_read_page(
            0,
            current_page,
            trajectory_flash_buffer,
            (uint16)words_to_read);
        byte_count = words_to_read * (uint32_t)sizeof(uint32_t);
        memcpy(
            (uint8_t *)Path_Memory +
                words_read * (uint32_t)sizeof(uint32_t),
            trajectory_flash_buffer,
            byte_count);
        words_read += words_to_read;
        current_page++;
    }

    if (words_read != total_words)
    {
        return trajectory_publish_failure(TRAJECTORY_DATA_INVALID);
    }

    actual_crc = trajectory_crc32(
        Path_Memory,
        meta.point_count * (uint32_t)sizeof(INS_Pos_t));
    if (actual_crc != meta.payload_crc32)
    {
        return trajectory_publish_failure(TRAJECTORY_CRC_ERROR);
    }

    if (!trajectory_path_data_valid(
            Path_Memory,
            meta.point_count,
            meta.total_mileage_ticks))
    {
        return trajectory_publish_failure(TRAJECTORY_DATA_INVALID);
    }

    min_target_speed = (int)meta.planned_min_speed;
    max_target_speed = (int)meta.planned_max_speed;
    DECEL_STEP = (int)meta.decel_step;
    ACCEL_STEP = (int)meta.accel_step;
    Total_Path_Points = (int)meta.point_count;
    Current_Path_Index = 0;
    trajectory_set_start_speed_to_max();
    trajectory_rebuild_drs_flags();
    trajectory_map_valid = 1U;
    path_record_full = (uint8_t)(meta.point_count >= MAX_PATH_POINTS);
    trajectory_last_result = TRAJECTORY_OK;
    (void)route_progress_rebuild_map();
    return TRAJECTORY_OK;
}

TrajectoryResult Save_Trajectory_To_Flash(void)
{
    TrajectoryMeta meta;
    uint32_t total_words;
    uint32_t words_written = 0U;
    uint32_t current_page = TRAJECTORY_DATA_FIRST_PAGE;

    trajectory_flush_final_interval();
    if (Total_Path_Points <= 0)
    {
        trajectory_last_result = TRAJECTORY_EMPTY;
        return TRAJECTORY_EMPTY;
    }
    if (Total_Path_Points > MAX_PATH_POINTS ||
        !trajectory_plan_params_valid(
            min_target_speed,
            max_target_speed,
            DECEL_STEP,
            ACCEL_STEP) ||
        !INS_Generate_Velocity_Table(min_target_speed, max_target_speed))
    {
        trajectory_last_result = TRAJECTORY_META_INVALID;
        return TRAJECTORY_META_INVALID;
    }

    meta.magic = TRAJECTORY_MAGIC;
    meta.format_version = TRAJECTORY_FORMAT_VERSION;
    meta.record_size = sizeof(INS_Pos_t);
    meta.point_count = (uint32_t)Total_Path_Points;
    meta.total_mileage_ticks =
        Path_Memory[Total_Path_Points - 1].mileage_ticks;
    meta.planned_min_speed = (uint32_t)min_target_speed;
    meta.planned_max_speed = (uint32_t)max_target_speed;
    meta.decel_step = (uint32_t)DECEL_STEP;
    meta.accel_step = (uint32_t)ACCEL_STEP;
    meta.payload_crc32 = trajectory_crc32(
        Path_Memory,
        (uint32_t)Total_Path_Points * (uint32_t)sizeof(INS_Pos_t));

    if (!trajectory_path_data_valid(
            Path_Memory,
            meta.point_count,
            meta.total_mileage_ticks))
    {
        trajectory_last_result = TRAJECTORY_DATA_INVALID;
        return TRAJECTORY_DATA_INVALID;
    }

    trajectory_invalidate_flash_map();

    total_words = (uint32_t)Total_Path_Points *
        (uint32_t)(sizeof(INS_Pos_t) / sizeof(uint32_t));
    while (words_written < total_words &&
           current_page <= TRAJECTORY_DATA_LAST_PAGE)
    {
        uint32_t words_to_write = total_words - words_written;
        uint32_t byte_count;

        if (words_to_write > EEPROM_PAGE_LENGTH)
        {
            words_to_write = EEPROM_PAGE_LENGTH;
        }
        byte_count = words_to_write * (uint32_t)sizeof(uint32_t);
        memcpy(
            trajectory_flash_buffer,
            (const uint8_t *)Path_Memory +
                words_written * (uint32_t)sizeof(uint32_t),
            byte_count);
        flash_write_page(
            0,
            current_page,
            trajectory_flash_buffer,
            (uint16)words_to_write);
        words_written += words_to_write;
        current_page++;
    }

    if (words_written != total_words)
    {
        trajectory_map_valid = 0U;
        trajectory_last_result = TRAJECTORY_WRITE_VERIFY_ERROR;
        route_progress_clear();
        return TRAJECTORY_WRITE_VERIFY_ERROR;
    }

    flash_write_page(
        0,
        TRAJECTORY_META_PAGE,
        (const uint32 *)&meta,
        (uint16)(sizeof(meta) / sizeof(uint32_t)));

    if (!trajectory_verify_flash_write(&meta))
    {
        trajectory_invalidate_flash_map();
        trajectory_map_valid = 0U;
        trajectory_last_result = TRAJECTORY_WRITE_VERIFY_ERROR;
        route_progress_clear();
        return TRAJECTORY_WRITE_VERIFY_ERROR;
    }

    Current_Path_Index = 0;
    trajectory_map_valid = 1U;
    trajectory_last_result = TRAJECTORY_OK;
    (void)route_progress_rebuild_map();
    return TRAJECTORY_OK;
}
