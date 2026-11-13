#ifndef CODE_INERTIAL_NAVIGATION_H_
#define CODE_INERTIAL_NAVIGATION_H_

#include "zf_common_headfile.h"
#include "state_control.h"

#include <stdint.h>

#define MAX_PATH_POINTS             3072
#define RECORD_INTERVAL_TICKS       1000U
#define TRAJECTORY_TARGET_SPEED_MAX 2000
#define TRAJECTORY_PEAK_ERROR_MAX   90
#define TRAJECTORY_STEP_MIN         1
#define TRAJECTORY_STEP_MAX         200
#define TRAJECTORY_DECEL_DEFAULT    15
#define TRAJECTORY_ACCEL_DEFAULT    50
#define TRAJECTORY_DRS_MIN_TICKS    25000U
#define TRAJECTORY_DRS_RECOVERY_TICKS 20000U

// 实时车辆状态，仅保存在 RAM 中。
typedef struct
{
    float mileage_ticks;
    float yaw;
    float filtered_error;
    int32_t error;
    uint8_t element_code;
} Car_State_t;

// Flash 轨迹点：固定 8 字节。
typedef struct
{
    uint32_t mileage_ticks;
    uint16_t target_speed;
    uint8_t element_code;
    int8_t peak_error;
} INS_Pos_t;

typedef char ins_pos_size_must_be_8[(sizeof(INS_Pos_t) == 8U) ? 1 : -1];

typedef enum
{
    TRAJECTORY_OK = 0,
    TRAJECTORY_EMPTY,
    TRAJECTORY_OLD_FORMAT,
    TRAJECTORY_META_INVALID,
    TRAJECTORY_CRC_ERROR,
    TRAJECTORY_DATA_INVALID,
    TRAJECTORY_WRITE_VERIFY_ERROR
} TrajectoryResult;

extern Car_State_t car_pos;
extern INS_Pos_t Path_Memory[MAX_PATH_POINTS];
extern int Total_Path_Points;
extern int Current_Path_Index;
extern int min_target_speed;
extern int max_target_speed;
extern int DECEL_STEP;
extern int ACCEL_STEP;
extern uint8_t trajectory_map_valid;
extern uint8_t path_record_full;
extern TrajectoryResult trajectory_last_result;
extern volatile uint32_t ins_relocalize_attempt_count;
extern volatile uint32_t ins_relocalize_success_count;
extern volatile int32_t ins_relocalize_last_correction_ticks;

void INS_Init(void);
void INS_Update(int enc_L, int enc_R, float imu_yaw);

void INS_Clear_Map(void);
void INS_Record_Trajectory(void);

uint8_t INS_Generate_Velocity_Table(int min_speed, int max_speed);

void INS_Sync_Tracking_Index(void);
int32_t INS_Get_Target_Speed(void);
uint8_t INS_Is_DRS_Eligible(void);
INS_Pos_t INS_Get_Lookahead_Point(float lookahead_dist_ticks);

void INS_Relocalize_At_Flag(int target_flag);

TrajectoryResult Save_Trajectory_To_Flash(void);
TrajectoryResult Load_Trajectory_From_Flash(void);

#endif /* CODE_INERTIAL_NAVIGATION_H_ */
