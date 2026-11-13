


/**************************************************************
 * 文件名:  motor.c
 * 创建时间: 2026年3月6日
 * 说明: 直流电机速度控制、PID结合路径偏差综合控制、无线发送
 **************************************************************/
#include "motor.h"
#include <stdio.h>                      // 用于 sprintf
#include "zf_device_wireless_uart.h"    // 用于无线串口发送
#include "isr_config.h"
#include "isr.h"
#include "inertial_navigation.h"
#include "line.h"
#include "run_features.h"
#include "state_control.h"
#include "menu.h"

// 硬件参数常量
#define PWM_FREQ_HZ        17000    // PWM频率
#define PWM_MAX            10000    // PWM最大值
#define PWM_MIN            0        // PWM最小值

// 电机堵转保护：大 PWM 且编码器速度很小时，持续 500 ms 后停车。
#define STALL_PROTECTION_ENABLE     1
#define STALL_PWM_THRESHOLD       5000
#define STALL_ENCODER_THRESHOLD   30
#define STALL_CONFIRM_CYCLES      250
// steer_output_max / encoder_max_val 改为变量，由 initialize.c 在 menu_init 后根据 basic_speed 赋值
float steer_output_max = 350.0f;
int encoder_max_val = 800;
#define SOFT_START_MIN     100       // 软启动初始值，低于此值时跳过加速过程
#define ACCEL_STEP         5         // 每次执行周期增加的速度步进值，值越小加速越平滑，但实际响应频率需同步调整
// DRS：长直道 MaxV 平台降低负压，减少直道阻力。
#define NEGATIVE_PRESSURE_DRS       55

float negative_pressure_percent = 50.0f; // 负压风扇百分比 (0~100), 菜单可调, Flash可存
static int negative_pressure_last_percent = -1;

// 转向差速的非对称补偿 (内外侧比例)
#define IN_SPD_PARAM  0.2f     // 内侧速度补偿系数
#define DE_SPD_PARAM  0.8f     // 外侧速度衰减系数

volatile int current_ramp_speed = 0;    // 当前实际基础速度
static uint8_t race_soft_start_active = 1U;

// 速度PID参数
PosPID pid_left = {0.0, 0.0f, 0.0f, 0, 0, 0, 0, 0.0f};
PosPID pid_right = {0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0.0f};
// 转向PD参数
SteeringPID turnPID = {0};

int dis_R = 0, dis_L = 0;
volatile int stop_flag = 0;
static volatile StopReason stop_reason = STOP_REASON_NONE;
static volatile uint8_t stop_reason_sent = 0U;

/* 每个三叉进入时记零点，退出时冻结该三叉内的编码器里程诊断数据。 */
static volatile int32_t fork_mileage_start_left = 0;
static volatile int32_t fork_mileage_start_right = 0;
static volatile int fork_exit_report_fork_index = 0;
static volatile int32_t fork_exit_report_mileage_difference = 0;
static volatile int32_t fork_exit_report_mileage_average = 0;
static volatile int fork_exit_report_angle_difference = 0;
static volatile uint32_t fork_exit_report_sequence = 0U;


int actual_right_speed = 0;
int actual_left_speed = 0;
int right_speed = 0;
int left_speed = 0;
int pwm_left = 0;
int pwm_right = 0;
int max_pwm_output = 0;
int steer_speed_val = 0;

const char *stop_reason_get_text(void)
{
    switch (stop_reason)
    {
        case STOP_REASON_TIMEOUT:        return "TIMEOUT";
        case STOP_REASON_ROUTE_END:      return "ROUTE_END";
        case STOP_REASON_LINE_LOST:      return "LINE_LOST";
        case STOP_REASON_PITCH_LIMIT:    return "PITCH_LIMIT";
        case STOP_REASON_LINE_AND_PITCH: return "LINE+PITCH";
        case STOP_REASON_STALL_LEFT:     return "STALL_LEFT";
        case STOP_REASON_STALL_RIGHT:    return "STALL_RIGHT";
        case STOP_REASON_STALL_BOTH:     return "STALL_BOTH";
        case STOP_REASON_YAW_PROTECTION: return "YAW_PROTECT";
        case STOP_REASON_NONE:
        default:                         return "UNKNOWN";
    }
}

void Motor_Protection_Check(void)
{
    static uint8_t left_stall_cycles = 0U;
    static uint8_t right_stall_cycles = 0U;
    int left_speed_abs;
    int right_speed_abs;

#if (STALL_PROTECTION_ENABLE == 0)
    left_stall_cycles = 0U;
    right_stall_cycles = 0U;
    return;
#endif

    if (set_f != 1 || stop_flag == 1) {
        left_stall_cycles = 0U;
        right_stall_cycles = 0U;
        return;
    }

    left_speed_abs = (actual_left_speed < 0) ? -actual_left_speed : actual_left_speed;
    right_speed_abs = (actual_right_speed < 0) ? -actual_right_speed : actual_right_speed;

    if (pwm_left >= STALL_PWM_THRESHOLD &&
        left_speed_abs <= STALL_ENCODER_THRESHOLD) {
        if (left_stall_cycles < STALL_CONFIRM_CYCLES) {
            left_stall_cycles++;
        }
    } else {
        left_stall_cycles = 0U;
    }

    if (pwm_right >= STALL_PWM_THRESHOLD &&
        right_speed_abs <= STALL_ENCODER_THRESHOLD) {
        if (right_stall_cycles < STALL_CONFIRM_CYCLES) {
            right_stall_cycles++;
        }
    } else {
        right_stall_cycles = 0U;
    }

    if (left_stall_cycles >= STALL_CONFIRM_CYCLES &&
        right_stall_cycles >= STALL_CONFIRM_CYCLES) {
        car_stop(STOP_REASON_STALL_BOTH);
    } else if (left_stall_cycles >= STALL_CONFIRM_CYCLES) {
        car_stop(STOP_REASON_STALL_LEFT);
    } else if (right_stall_cycles >= STALL_CONFIRM_CYCLES) {
        car_stop(STOP_REASON_STALL_RIGHT);
    }
}

// 专门用于清零瞬时累积历史状态
void Motor_Reset_State(void) {
    // 左轮位置式PID
    pid_left.error = pid_left.last_error = pid_left.prev_error = 0;
    pid_left.integral = 0.0f;

    // 右轮位置式PID
    pid_right.error = pid_right.last_error = pid_right.prev_error = 0;
    pid_right.integral = 0.0f;

    // 转向PID
    turnPID.error = turnPID.last_error = turnPID.prev_error = 0;

    current_ramp_speed = 0;
    max_pwm_output = 0;
    race_soft_start_active = 1U;
    stop_reason = STOP_REASON_NONE;
    stop_reason_sent = 0U;
}

// 无刷电调 PWM 参数：50Hz，1ms~2ms 脉宽
// duty 范围 500(1ms) ~ 1000(2ms)，对应 PWM_DUTY_MAX=10000
#define ESC_PWM_FREQ        (50)
#define ESC_DUTY_MIN        (500)   // 1ms 电调怠速/解锁
#define ESC_DUTY_MAX        (1000)  // 2ms 满油门

// 把 0~100 映射到 ESC_DUTY_MIN~ESC_DUTY_MAX
#define ESC_PERCENT_TO_DUTY(pct)  \
    ((int)(ESC_DUTY_MIN + ((pct) * (ESC_DUTY_MAX - ESC_DUTY_MIN) / 100)))

void negative_pressure_init(){
    pwm_init(ATOM0_CH0_P22_1, ESC_PWM_FREQ, 0);
    //gpio_init(P23_1, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void negative_pressure_set_speed(int speed_percent)
{
    if(speed_percent < 0)   speed_percent = 0;
    if(speed_percent > 100) speed_percent = 100;
    pwm_set_duty(ATOM0_CH0_P22_1, ESC_PERCENT_TO_DUTY(speed_percent));
}

void negative_pressure_start(){
    int target_percent = (int)negative_pressure_percent;
    int step = 2;
    int current_percent;

    for (current_percent = 0; current_percent <= target_percent; current_percent += step)
    {
        negative_pressure_set_speed(current_percent);
        system_delay_ms(1);  // 1ms 步进，让电调平滑响应
    }
    negative_pressure_set_speed(target_percent);
    negative_pressure_last_percent = target_percent;
}

void negative_pressure_stop(){
    pwm_set_duty(ATOM0_CH0_P22_1,0);  // 回到 1ms 怠速，不要直接给 0
    negative_pressure_last_percent = -1;
    //gpio_set_level(P23_1, 0);
}

void negative_pressure_update(int planned_speed)
{
    int target_percent = (int)negative_pressure_percent;

    if (stop_flag == 1)
    {
        return;
    }

    if (drs_enable &&
        speed_decision_enable &&
        planned_speed == max_target_speed &&
        INS_Is_DRS_Eligible() &&
        target_percent > NEGATIVE_PRESSURE_DRS)
    {
        target_percent = NEGATIVE_PRESSURE_DRS;
    }

    if (target_percent != negative_pressure_last_percent)
    {
        negative_pressure_set_speed(target_percent);
        negative_pressure_last_percent = target_percent;
    }
}

/**************************************************************
 * NSLEEP 控制实现（适配 DRV8245-Q1 HW 版本 VQFN 封装）
 * 引脚：P33.6 输出控制电机驱动芯片睡眠状态
 *       P33.7 按键输入，按下时关断 NSLEEP 使驱动休眠
 *
 * DRV8245-Q1 时序要求：
 *   tWAKEUP  = 10 us (max)   NSLEEP 高电平保持时间唤醒滤波
 *   tSLEEP   = 40~120 us     NSLEEP 低电平保持时间进入休眠 (HW)
 *   tRESET   = 5~20 us       NSLEEP 低电平脉冲清除故障范围 (HW)
 *   tREADY   = 1 ms (max)    唤醒后驱动就绪时间
 *   tCOM     = 400 us (max)  唤醒后 SPI 通信可用时间
 *
 * HW 版本上电握手时序：
 *   POR 完成后，MCU 必须发出 nSLEEP 低电平脉冲（30~40us）
 *   作为对芯片的确认握手，之后芯片才进入 STANDBY 可操作
 **************************************************************/

#define NSLEEP_T_WAKEUP_US    10      // tWAKEUP：高电平唤醒滤波时间 (us)
#define NSLEEP_T_READY_MS     1       // tREADY：唤醒后等待驱动就绪 (ms)
#define NSLEEP_HW_RESET_US    30      // HW 版本上电握手脉冲宽度 (30us)

void NSLEEP_Init(void)
{
    // === HW 版本上电握手序列 ===
    // nSLEEP 被内部下拉保持低电平，芯片处于 SLEEP 状态

    // 1. 拉高 nSLEEP，退出 SLEEP（产生上升沿触发）
    gpio_init(NSLEEP_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_us(NSLEEP_T_WAKEUP_US);

    // 2. 发出 30us 低电平握手脉冲
    gpio_set_level(NSLEEP_PIN, GPIO_LOW);
    system_delay_us(NSLEEP_HW_RESET_US);
    gpio_set_level(NSLEEP_PIN, GPIO_HIGH);

    // 3. 等待芯片完成初始化进入 STANDBY/ACTIVE
    system_delay_us(NSLEEP_T_WAKEUP_US);
    system_delay_ms(NSLEEP_T_READY_MS);

    // NSLEEP 按键输入：内部上拉，按下为低电平
    gpio_init(NSLEEP_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void NSLEEP_Wake(void)
{
    gpio_set_level(NSLEEP_PIN, GPIO_LOW);
    system_delay_us(NSLEEP_T_WAKEUP_US);

    gpio_set_level(NSLEEP_PIN, GPIO_HIGH);
    system_delay_us(NSLEEP_T_WAKEUP_US);
    system_delay_ms(NSLEEP_T_READY_MS);
}

void NSLEEP_Sleep(void)
{
    L_set_PWM(0);
    R_set_PWM(0);
    pwm_left = 0;
    pwm_right = 0;

    gpio_set_level(NSLEEP_PIN, GPIO_LOW);
    system_delay_us(50);
}

void NSLEEP_ClearFault(void)
{
    gpio_set_level(NSLEEP_PIN, GPIO_LOW);
    system_delay_us(NSLEEP_HW_RESET_US);
    gpio_set_level(NSLEEP_PIN, GPIO_HIGH);
    system_delay_us(NSLEEP_T_WAKEUP_US);
    system_delay_ms(NSLEEP_T_READY_MS);
}

void NSLEEP_Key_Handle(void)
{
    if (gpio_get_level(NSLEEP_KEY) == 0)
    {
        system_delay_ms(20);
        if (gpio_get_level(NSLEEP_KEY) == 0)
        {
            while (gpio_get_level(NSLEEP_KEY) == 0);
            system_delay_ms(20);
            NSLEEP_Sleep();
        }
    }
}

/**************************************************************
 * 函数: 初始化GPIO/PWM/编码器
 **************************************************************/
void Motor_Init(void)
{
    // 右电机方向引脚
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 右电机PWM引脚初始化
    pwm_init(PWM_R1, PWM_FREQ_HZ,0);

    // 左电机方向引脚初始化
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 左电机PWM引脚初始化
    pwm_init(PWM_L1, PWM_FREQ_HZ,0);

    // 初始化转向PID历史状态
    turnPID.error = turnPID.last_error = turnPID.prev_error = 0.0f;
    /////////负压初始化////////
    negative_pressure_init();
}

/**************************************************************
 * 函数: 设置左电机PWM值
 **************************************************************/
void L_set_PWM(int Speed)
{
    // 约束PWM范围
    int pwm = constrain(Speed, PWM_MIN, PWM_MAX);

    // 设置方向与速度
    if (pwm > 0)
    {
        gpio_set_level(DIR_L1, GPIO_HIGH);
        pwm_set_duty(PWM_L1, pwm);
    }
//    else if (pwm < 0)
//    {
//        gpio_set_level(DIR_L1, GPIO_LOW);
//        pwm_set_duty(PWM_L1, -pwm);
//    }
//    if (pwm < 0)
//        {
//            gpio_set_level(DIR_L1, GPIO_LOW);
//            pwm_set_duty(PWM_L1, -pwm);
//        }
    else
    {
        pwm_set_duty(PWM_L1, 0);
    }
}

/**************************************************************
 * 函数: 设置右电机PWM值
 **************************************************************/
void R_set_PWM(int Speed)
{
    // 约束PWM范围
    int pwm = constrain(Speed, PWM_MIN, PWM_MAX);

    // 设置方向与速度
    if (pwm > 0)
    {
        gpio_set_level(DIR_R1, GPIO_HIGH);
        pwm_set_duty(PWM_R1, pwm);
    }
//    else if (pwm < 0)
//    {
//        gpio_set_level(DIR_R1, GPIO_LOW);
//        pwm_set_duty(PWM_R1, -pwm);
//    }
    else
    {
        pwm_set_duty(PWM_R1, 0);
    }
}
float K_forward = 0.0f;   // 速度前馈系数，用于克服摩擦和反电动势

/**************************************************************
 * 函数: 左轮速度控制 (PID + 速度前馈)
 **************************************************************/
void speed_control_L(int target_speed_L)
{
    pid_left.target = target_speed_L;
    actual_left_speed = -Get_Encoder_L();
    dis_L += actual_left_speed;

    // 1. 计算速度PID；传入前馈和最终PWM范围用于积分抗饱和判断。
    float feedforward = K_forward * target_speed_L;
    int pi_pwm = PosPID_Calculate(&pid_left,
                                  actual_left_speed,
                                  feedforward,
                                  PWM_MIN,
                                  PWM_MAX);

    // 2. 总输出 = PID输出 + 速度前馈
    int total_pwm = pi_pwm + (int)feedforward;

    total_pwm = constrain(total_pwm, PWM_MIN, PWM_MAX);

    if(stop_flag == 1){
        L_set_PWM(0);
        pwm_left = 0;
    }else{
        L_set_PWM(total_pwm);
        pwm_left = total_pwm;
        if (total_pwm > max_pwm_output) {
            max_pwm_output = total_pwm;
        }
    }
}

/**************************************************************
 * 函数: 右轮速度控制 (PID + 速度前馈)
 **************************************************************/
void speed_control_R(int target_speed_R)
{
    pid_right.target = target_speed_R;
    actual_right_speed = Get_Encoder_R();
    dis_R += actual_right_speed;

    // 1. 计算速度PID；传入前馈和最终PWM范围用于积分抗饱和判断。
    float feedforward = K_forward * target_speed_R;
    int pi_pwm = PosPID_Calculate(&pid_right,
                                  actual_right_speed,
                                  feedforward,
                                  PWM_MIN,
                                  PWM_MAX);

    // 2. 总输出 = PID输出 + 速度前馈
    int total_pwm = pi_pwm + (int)feedforward;

    total_pwm = constrain(total_pwm, PWM_MIN, PWM_MAX);

    if(stop_flag == 1){
        R_set_PWM(0);
        pwm_right = 0;
    }else{
        R_set_PWM(total_pwm);
        pwm_right = total_pwm;
        if (total_pwm > max_pwm_output) {
            max_pwm_output = total_pwm;
        }
    }
}

static void motor_soft_start_toward(int target_speed)
{
    if (current_ramp_speed < SOFT_START_MIN &&
        target_speed >= SOFT_START_MIN)
    {
        current_ramp_speed = SOFT_START_MIN;
    }
    else if (current_ramp_speed < target_speed)
    {
        current_ramp_speed += ACCEL_STEP;
        if (current_ramp_speed > target_speed)
        {
            current_ramp_speed = target_speed;
        }
    }
    else if (current_ramp_speed > target_speed)
    {
        current_ramp_speed = target_speed;
    }
}

/**************************************************************
 * 函数: 综合控制 (基础速度 + 路径偏差补偿 + 转向差速)
 **************************************************************/
void combined_control(int target_base_speed, int path_deviation)
{

    static uint8_t control_div_cnt = 0;
    int planned_speed_for_drs = -1;

    // --- 1. 偏差低通滤波 ---
    static float filtered_deviation = 0.0f;
    const float alpha = 1.0f;
    filtered_deviation = alpha * path_deviation + (1.0f - alpha) * filtered_deviation;

    // =========================================================================
    // --- 3. 目标速度斜坡 (逐步加速的软启动) ---
    int base_speed;

    // 限定目标速度范围，防止超速
    target_base_speed = constrain(target_base_speed, 0, encoder_max_val);

    if (stop_flag == 1 || target_base_speed == 0) {
        // 停车状态，速度清零
        current_ramp_speed = 0;
    } else {
        if (speed_decision_enable) {
            int planned_speed = INS_Get_Target_Speed();
            planned_speed_for_drs = planned_speed;

            if (race_soft_start_active)
            {
                motor_soft_start_toward(planned_speed);
                if (current_ramp_speed >= planned_speed)
                {
                    current_ramp_speed = planned_speed;
                    race_soft_start_active = 0U;
                }
            }
            else
            {
                current_ramp_speed = planned_speed;
            }
        }
        else {
            // 不启用速度决策时继续使用基础速度和原有缓启动规律。
            motor_soft_start_toward(target_base_speed);
        }
    }

    // 将平滑后的速度作为实际给定速度
    base_speed = current_ramp_speed;
    // =========================================================================

    // === [降频] 降频检测逻辑：3帧跑1帧 ===
    control_div_cnt++;
    if (control_div_cnt >= 2)
    {
        control_div_cnt = 0; // 计数器复位

        // --- 2. 模糊PD转向控制 (降频执行，节省算力) ---
        float steer_speed = steering_fuzzy_pd_control(&turnPID, 0.0f, filtered_deviation);
        steer_speed = constrain(steer_speed, -steer_output_max, steer_output_max);
        steer_speed_val = (int)steer_speed;

        // --- 4. 差速分配 ---
        if (steer_speed > 0)
        {
            right_speed = base_speed - (int) (DE_SPD_PARAM * steer_speed);
            left_speed  = base_speed + (int) (IN_SPD_PARAM * steer_speed);
        }
        else
        {
            right_speed = base_speed - (int)(IN_SPD_PARAM * steer_speed);
            left_speed  = base_speed + (int)(DE_SPD_PARAM * steer_speed);
        }

        // --- 防止速度越界 ---
        left_speed  = constrain(left_speed, -encoder_max_val, encoder_max_val);
        right_speed = constrain(right_speed, -encoder_max_val, encoder_max_val);
    }


    speed_control_L(left_speed);
    speed_control_R(right_speed);
    Motor_Protection_Check();
    negative_pressure_update(planned_speed_for_drs);
//    ips114_show_float(0,16,actual_left_speed,3,3);
//    ips114_show_float(0,40,actual_right_speed,3,3);
    // --- 6. 每帧执行后更新实时位置 ---
    INS_Update(actual_left_speed, actual_right_speed, get_yaw());
}
/**************************************************************
 * 函数: 封装并发送电机数据到无线串口上位机
 * 格式: 三叉编号,左目标,右目标,左实际,右实际,路径偏差\r\n
 **************************************************************/
void send_speed_to_pc(void)
{
    // 格式: 三叉编号,目标速度左,目标速度右,实际速度左,实际速度右,路径偏差\r\n
    char send_buf[80];

    sprintf(send_buf, "%d,%d,%d,%d,%d,%d\r\n",
            car_state == normal ? 0 : fork_num,
            left_speed, right_speed,
            actual_left_speed, actual_right_speed,
            return_error()
          );

    wireless_uart_send_string(send_buf);
}

void send_stop_reason_to_pc(void)
{
    char send_buf[32];

    if (stop_flag != 1 || stop_reason_sent)
    {
        return;
    }

    (void)sprintf(send_buf, "STOP,%s\r\n", stop_reason_get_text());
    wireless_uart_send_string(send_buf);
    stop_reason_sent = 1U;
}

void fork_mileage_report_begin(void)
{
    fork_mileage_start_left = dis_L;
    fork_mileage_start_right = dis_R;
}

void fork_exit_mileage_report_capture(int fork_index, int angle_difference)
{
    int32_t left_mileage = dis_L - fork_mileage_start_left;
    int32_t right_mileage = dis_R - fork_mileage_start_right;

    fork_exit_report_fork_index = fork_index;
    fork_exit_report_mileage_difference = left_mileage - right_mileage;
    fork_exit_report_mileage_average = (left_mileage + right_mileage) / 2;
    fork_exit_report_angle_difference = angle_difference;
    fork_exit_report_sequence++;
}

void send_fork_exit_mileage_report_to_pc(void)
{
    static uint32_t sent_sequence = 0U;
    uint32_t sequence_before;
    uint32_t sequence_after;
    int fork_index;
    int32_t mileage_difference;
    int32_t mileage_average;
    int angle_difference;
    char send_buf[64];

    sequence_before = fork_exit_report_sequence;
    if (sequence_before == sent_sequence)
    {
        return;
    }

    fork_index = fork_exit_report_fork_index;
    mileage_difference = fork_exit_report_mileage_difference;
    mileage_average = fork_exit_report_mileage_average;
    angle_difference = fork_exit_report_angle_difference;
    sequence_after = fork_exit_report_sequence;
    if (sequence_before != sequence_after)
    {
        return;
    }

    sent_sequence = sequence_before;
    sprintf(send_buf, "%d,%ld,%ld,%d\r\n",
            fork_index,
            (long)mileage_difference,
            (long)mileage_average,
            angle_difference);
    wireless_uart_send_string(send_buf);
}

void car_stop(StopReason reason){
    if (stop_reason == STOP_REASON_NONE)
    {
        stop_reason = reason;
    }
    stop_flag = 1;
    NSLEEP_Sleep();  // 立即关断电机驱动，阻止在途控制 ISR 的旧 PWM 再次带动车轮
    if (record_enable) {
        menu_request_map_save_prompt();
    }
    pit_disable(CCU60_CH0);
    L_set_PWM(0);
    R_set_PWM(0);
    system_delay_ms(500);
    negative_pressure_stop();
}
