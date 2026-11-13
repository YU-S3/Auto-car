#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"
#include"tool.h"

// 速度环积分仅用于补偿稳态误差：阈值按各轮目标速度动态计算。
// 原实现使用固定分离阈值 500、积分累加值限幅 2000，且大误差时直接清零。
#define PID_E_FULL_RATIO            0.05f
#define PID_E_OFF_RATIO             0.25f
#define PID_E_FULL_MIN              20.0f
#define PID_E_FULL_MAX              100.0f
#define PID_E_OFF_MIN               60.0f
#define PID_E_OFF_MAX               300.0f
#define PID_I_OUTPUT_MAX            1000.0f
#define PID_I_LEAK_FACTOR           0.995f
#define PID_LOW_SPEED_THRESHOLD     50

typedef enum {
    STOP_REASON_NONE = 0,
    STOP_REASON_TIMEOUT,
    STOP_REASON_ROUTE_END,
    STOP_REASON_LINE_LOST,
    STOP_REASON_PITCH_LIMIT,
    STOP_REASON_LINE_AND_PITCH,
    STOP_REASON_STALL_LEFT,
    STOP_REASON_STALL_RIGHT,
    STOP_REASON_STALL_BOTH,
    STOP_REASON_YAW_PROTECTION
} StopReason;

// 硬件引脚定义
#define DIR_R1             (P33_9)
#define PWM_R1              (ATOM3_CH0_P33_10)
#define DIR_L1               (P33_11)
#define PWM_L1              (ATOM3_CH4_P33_12)


// NSLEEP 电机驱动休眠控制引脚
#define NSLEEP_PIN          (P33_6)     // NSLEEP 输出：高电平唤醒，低电平休眠
#define NSLEEP_KEY          (P33_7)     // NSLEEP 按键输入：按下可关断 NSLEEP


// ************************* 全局变量声明 *************************
extern volatile  int stop_flag;
extern float steer_output_max;
extern int encoder_max_val;
extern float basic_speed;
extern volatile int current_ramp_speed;
extern int max_pwm_output;
extern float K_forward;          // 速度前馈系数
extern float negative_pressure_percent;  // 负压风扇百分比 (0~100)


// ************************* 函数声明 *************************
void Motor_Init(void);
void L_set_PWM(int Speed);
void R_set_PWM(int Speed);
void speed_control_L(int target_speed_L);
void speed_control_R(int target_speed_R);
void combined_control(int base_speed, int path_deviation);
void Motor_Protection_Check(void);
void send_speed_to_pc(void);
void send_stop_reason_to_pc(void);
const char *stop_reason_get_text(void);
void fork_mileage_report_begin(void);
void fork_exit_mileage_report_capture(int fork_index, int angle_difference);
void send_fork_exit_mileage_report_to_pc(void);
void car_stop(StopReason reason);
void negative_pressure_init();
void negative_pressure_start();
void negative_pressure_stop();
void negative_pressure_set_speed(int speed_percent);
void negative_pressure_update(int planned_speed);
void Motor_Reset_State(void);

// NSLEEP 控制函数
void NSLEEP_Init(void);         // 初始化 NSLEEP 引脚（默认唤醒）
void NSLEEP_Wake(void);         // 唤醒电机驱动
void NSLEEP_Sleep(void);        // 休眠电机驱动
void NSLEEP_ClearFault(void);   // 脉冲 NSLEEP 清除故障
void NSLEEP_Key_Handle(void);   // 检测按键并关断 NSLEEP



#endif /* CODE_MOTOR_H_ */
