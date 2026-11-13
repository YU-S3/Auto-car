/**************************************************************
 * 文件名:  tool.c
 * 创建时间: 2026年2月19日  下午9:55:57
 * 说明: 通用工具函数、PID算法、滤波、图像显示等
 * 2026.4.7 适配IPS114屏幕
 **************************************************************/
//#include "zf_common_headfile.h"
#include "tool.h"
#include "motor.h"
#include "state_control.h"
#include "zf_device_ips114.h"   // 包含 IPS114 屏幕

extern float GZ;  // MahonyAHRS 中已低通滤波的 Z 轴角速度 (DPS)

// 全局时间步进，用于PID运算周期
float dt = 0.006f;

/**************************************************************
 * 函数: 显示红色像素点
 * 参数: x/y - 像素坐标
 * 说明: 自动裁剪范围防止越界
 **************************************************************/
void show_point_red(int x, int y)
{
    if (x > 0 && x < MT9V03X_W && y > 0 && y < MT9V03X_H)
    {
        ips114_draw_point((uint16)x, (uint16)y, RGB565_RED);
    }
}

/**************************************************************
 * 函数: 显示蓝色像素点
 * 参数: x/y - 像素坐标
 **************************************************************/
void show_point_blue(int x, int y)
{
    if (x > 0 && x < MT9V03X_W && y > 0 && y < MT9V03X_H)
    {
        ips114_draw_point((uint16)x, (uint16)y, RGB565_BLUE);
    }
}

/**************************************************************
 * 函数: 显示黄色像素点
 * 参数: x/y - 像素坐标
 **************************************************************/
void show_point_yellow(int x, int y)
{
    if (x > 0 && x < MT9V03X_W && y > 0 && y < MT9V03X_H)
    {
        ips114_draw_point((uint16)x, (uint16)y, RGB565_YELLOW);
    }
}

/**************************************************************
 * 函数: 绘制十字准星
 * 参数: x/y - 像素坐标
 **************************************************************/
void show_point(int x, int y)
{
    // 入参已缩放至显示区 (135×86), 非相机尺寸, 用显示区做边界检查
    if (x > 10 && x < 135 - 10 && y > 10 && y < 86 - 10)
    {
        for(int i=-1;i<2;i++){
        ips114_draw_line((uint16)x-10, (uint16)y+i,(uint16)x+10, (uint16)y+i,RGB565_RED);
        ips114_draw_line((uint16)x+i, (uint16)y-10,(uint16)x+i, (uint16)y+10,RGB565_RED);
        }
    }
}

/**************************************************************
 * 函数: 位置式PD控制器 (用于转向控制)
 * 参数: pid - PD控制器结构体指针
 *       setpoint - 目标值 (路径偏差目标)
 *       actual - 实际值 (当前路径偏差)
 * 返回: PD输出值 (叠加后的PWM调整量)
 * 公式: output = Kp*e(t) + Kd*[e(t)-e(t-1)]/dt
 **************************************************************/
float steering_pd_control(SteeringPID *pid, float setpoint, float actual)
{
    // 计算当前误差
    float current_error = setpoint - actual;
    float gyro_z = -GZ;  // NED坐标系取反, GZ已在Mahony中低通滤波

    // 1. 计算原始微分项 (变化率)
    float raw_derivative = (current_error - pid->last_error) / dt;

    // 2. 低通滤波平滑微分噪声，防止微分项突变抖动
    // 专为 5ms 降频设计的低通系数，防止车头/车身抖动导致的猛打角
    static float filtered_derivative = 0.0f;
    // 0.3偏响应，0.7偏平滑；增大滤波系数则响应更慢，减小(如0.15)则更灵敏
    filtered_derivative = 0.6f * raw_derivative + 0.4f * filtered_derivative;

    // 更新历史误差
    pid->last_error = current_error;
    pid->error = current_error;

    // 3. PD输出计算 (注意这里使用滤波后的 filtered_derivative)
    float output = pid->Kp * current_error + pid->Kd * filtered_derivative + pid->GKD * gyro_z;

    return output;
}

/**************************************************************
 * 函数: 位置式PID计算（用于速度控制）
 * 参数: pid           - 位置式PID控制器结构体指针
 *       current_speed - 当前实际速度（编码器读数）
 *       feedforward   - 本轮速度前馈PWM
 *       output_min/max- 电机最终PWM上下限
 * 返回: 不含前馈的PID修正量
 * 说明:
 *   1. 按本轮目标速度动态计算 E_full/E_off，并线性改变积分速度；
 *   2. 大误差时停止新增积分：同向积分保持，反向积分缓慢泄放；
 *   3. 直接限制积分能够贡献的PWM，避免Ki改变后积分权限失控；
 *   4. 根据“前馈 + PID”的最终候选输出进行条件积分，防止执行器饱和。
 *
 * 原实现备忘: 固定阈值500以内直接累加误差，累加值限幅±2000；
 *              阈值外立即清零积分，且积分计算时不知道最终PWM是否饱和。
 **************************************************************/
int PosPID_Calculate(PosPID *pid,
                     int current_speed,
                     float feedforward,
                     float output_min,
                     float output_max)
{
    if (pid == NULL) return 0;

    int error = pid->target - current_speed;
    float p_term = pid->Kp * error;
    float d_term = pid->Kd * (error - pid->last_error);
    float i_term = 0.0f;

    if (pid->Ki > 0.0f && abs(pid->target) >= PID_LOW_SPEED_THRESHOLD) {
        float abs_target = (float)abs(pid->target);
        float abs_error = (float)abs(error);
        float e_full = constrain(abs_target * PID_E_FULL_RATIO,
                                 PID_E_FULL_MIN,
                                 PID_E_FULL_MAX);
        float e_off = constrain(abs_target * PID_E_OFF_RATIO,
                                PID_E_OFF_MIN,
                                PID_E_OFF_MAX);
        float integral_scale;
        float current_i_term;
        float candidate_i_term;
        float candidate_output;
        uint8_t accept_candidate = 1U;

        if (abs_error <= e_full) {
            integral_scale = 1.0f;
        } else if (abs_error < e_off) {
            integral_scale = (e_off - abs_error) / (e_off - e_full);
        } else {
            integral_scale = 0.0f;
        }

        current_i_term = constrain(pid->Ki * pid->integral,
                                   -PID_I_OUTPUT_MAX,
                                   PID_I_OUTPUT_MAX);

        if (integral_scale > 0.0f) {
            candidate_i_term = current_i_term + pid->Ki * (float)error * integral_scale;
        } else if ((float)error * current_i_term > 0.0f) {
            // 大误差时保留方向正确的负载补偿，避免弯中欠速把已有积分持续泄掉。
            candidate_i_term = current_i_term;
        } else {
            // 积分方向与当前误差相反时缓慢退场，避免目标突变后旧积分继续推动车轮。
            candidate_i_term = current_i_term * PID_I_LEAK_FACTOR;
        }
        candidate_i_term = constrain(candidate_i_term,
                                     -PID_I_OUTPUT_MAX,
                                     PID_I_OUTPUT_MAX);

        candidate_output = feedforward + p_term + d_term + candidate_i_term;
        if (candidate_output > output_max && candidate_i_term > current_i_term) {
            accept_candidate = 0U;
        } else if (candidate_output < output_min && candidate_i_term < current_i_term) {
            accept_candidate = 0U;
        }

        i_term = accept_candidate ? candidate_i_term : current_i_term;
        pid->integral = i_term / pid->Ki;
    } else {
        // 停车、极低速或关闭Ki时不保留旧积分补偿。
        pid->integral = 0.0f;
    }

    int output = (int)(p_term + i_term + d_term);

    pid->prev_error = pid->last_error;
    pid->last_error = error;
    pid->error = error;

    return output;
}


float angle_difference(float angle1, float angle2) {
    // 计算直接差值
    float diff = fabs(angle1 - angle2);

    // 如果差值超过180度，取360-差值为最小角度差
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }

    return diff;
}
// ─── 线性跟踪微分器(LTD)参数 ─────────────────────────────────
// 用二阶线性低通滤波器替代原来的 fhan 非线性TD,
// 每个参数有独立物理意义, 不再相互耦合, 大幅降低调参难度.
//
// 调参口诀:
//   td_freq — 管"跟多紧": 直道画龙就减小(比如10Hz), 弯道跟不上就增大(比如20Hz)
//   td_damp — 管"稳不稳": 1.0=最稳定无超调, 0.7~0.85=稍快但允许一点过冲
//   dead_zone— 管"别乱动": 直道画龙就增大, 弯道反应迟钝就减小
//   steer_Kp2— 管"大弯猛不猛": 通常0即可, 大弯速度不够时加0.01~0.05
// ────────────────────────────────────────────────────────────
float td_freq   = 15.0f;   // TD自然频率(Hz): 5~25, 越大跟踪越快
float td_damp   = 1.0f;    // TD阻尼比: 1.0临界阻尼, 0.7轻微超调
float dead_zone = 5.0f;    // 死区(像素): 平滑偏差在此范围内不输出转向
float steer_Kp2 = 0.00f;   // 平方非线性系数: 大弯额外加力, 通常0即可

// ─── 线性跟踪微分器核心 ──────────────────────────────────────
// 二阶低通滤波器形式, 同时输出:
//   *y  → 平滑后的信号 (替代原 td_v1)
//   *dy → 信号的微分   (替代原 td_v2)
// 物理意义: omega = 2π*freq, 两个参数独立
//   d(y)/dt  = dy
//   d(dy)/dt = ω²*(u - y) - 2*ζ*ω*dy
// ────────────────────────────────────────────────────────────
static void linear_td_update(float *y, float *dy, float u,
                              float freq, float damp, float h)
{
    float omega   = 2.0f * 3.14159265f * freq;   // ω = 2πf
    float omega2  = omega * omega;
    // 二阶状态空间更新: 加速度 = 弹性恢复力 - 阻尼力
    float ddy     = omega2 * (u - *y) - 2.0f * damp * omega * (*dy);
    *dy          += h * ddy;
    *y           += h * (*dy);
}

float steering_fuzzy_pd_control(SteeringPID *pid, float setpoint, float actual)
{
    float current_error = setpoint - actual;
    float abs_e = fabsf(current_error);
    float gyro_z = -GZ;  // NED坐标系取反, GZ已在Mahony中低通滤波
    float gyro_damping_scale = state_get_turn_damping_scale();

    // ── 1. 线性跟踪微分器 ─────────────────────────────────
    static float td_v1 = 0.0f, td_v2 = 0.0f;
    static uint8_t td_inited = 0;
    float h = 0.004f;  // 实际控制周期 4ms
    if (!td_inited) {
        td_v1 = actual;
        td_v2 = 0.0f;
        td_inited = 1;
    }
    linear_td_update(&td_v1, &td_v2, actual, td_freq, td_damp, h);

    // ── 2. 死区: 平滑偏差在死区内 → 只留陀螺仪阻尼 ─────────
    //     这是防止直道画龙最关键的一环, 利用TD的滞后特性
    //     提供自然的迟滞, 避免在死区边界来回跳动
    if (fabsf(td_v1) <= dead_zone) {
        pid->last_error = current_error;
        pid->last_actual = actual;
        pid->error = current_error;
        return pid->GKD * gyro_damping_scale * gyro_z;
    }

    // 更新历史状态
    pid->last_error = current_error;
    pid->last_actual = actual;
    pid->error = current_error;

    // ── 3. PD + 非线性 + 陀螺仪 ────────────────────────────
    float p_linear = pid->Kp * current_error;
    float p_square = steer_Kp2 * current_error * abs_e;
    float d_term   = pid->Kd * (-td_v2);   // 平滑微分阻尼, 负号产生恢复力
    float g_term   = pid->GKD * gyro_damping_scale * gyro_z;

    return p_linear + p_square + d_term + g_term;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数名称      整数转字符串 (完整实现)
//  返回值       const char* 静态缓冲区地址
//  参数说明      value 要转换的整数值
//  使用示例      const char* str = int_to_str(-12345);
//  注意：       返回静态缓冲区地址，每次调用会覆盖之前的内容
//              支持最多10位数字 (含负号)
//-------------------------------------------------------------------------------------------------------------------
const char* int_to_str(int value) {

    static char buffer[12];

    int len = snprintf(buffer, sizeof(buffer), "%d", value);

    if (len < 0 || len >= sizeof(buffer)) {
        return "ERR";
    }

    return buffer;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数名称      发送两个整数到无线串口
//  返回值       void
//  参数说明      a 第一个整数值
//              b 第二个整数值
//  使用示例      sent_two_int(current_error, pwm_output);
//  发送格式      "a,b\n"
//-------------------------------------------------------------------------------------------------------------------
void sent_two_int(int a,int b){
    wireless_uart_send_string(int_to_str(a));
    wireless_uart_send_string(",");
    wireless_uart_send_string(int_to_str(b));
    wireless_uart_send_string("\n");
}
