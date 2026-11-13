#ifndef CODE_TOOL_H_
#define CODE_TOOL_H_

#include "zf_common_headfile.h"
void show_point_red(int x,int y);
void show_point_blue(int x,int y);
void show_point_yellow(int x,int y);
void show_point(int x,int y);

extern float dt;

// 线性跟踪微分器(TD)参数 — 二阶低通滤波器形式，参数物理意义独立
extern float td_freq;    // TD自然频率(Hz): 越大跟踪越快也越容易受噪声影响, 建议 5~25
extern float td_damp;    // TD阻尼比: 1.0=临界阻尼(无超调最平滑), 0.7=轻微超调略快
extern float dead_zone;  // 死区(像素): 偏差绝对值小于此值时不输出转向, 直道防画龙
extern float steer_Kp2;  // 平方非线性系数: 大弯时额外加力, 通常 0 即可

// 位置式PID结构体（速度环）
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    int target;             // 目标值
    int error;              // 当前误差 e(k)
    int last_error;         // 上一次误差 e(k-1)
    int prev_error;    // 上上次误差 e(k-2)
    float integral;         // 积分累积项（带积分分离）
} PosPID;



// 转向PID结构体（方向环）
typedef struct {
    float Kp;               // 比例系数
    float Kd;               // 微分系数
    float GKD;              // 陀螺仪角速度系数
    float error;            // 当前偏差
    float last_error;       // 上一次偏差
    float prev_error;  // 上上次偏差
    float last_actual;      // 上一次实际值（微分先行）
} SteeringPID;

typedef struct {
    float alpha; // 滤波系数
    float previous_output; // 上一次的输出值
} LowPassFilter;

// 位置式PD控制
float steering_pd_control(SteeringPID *pid, float setpoint, float actual);
float steering_fuzzy_pd_control(SteeringPID *pid, float setpoint, float actual);
// 位置式PID控制：返回不含前馈的 PID 修正量，内部依据最终执行器输出进行抗饱和。
extern int PosPID_Calculate(PosPID *pid,
                            int current_speed,
                            float feedforward,
                            float output_min,
                            float output_max);

float angle_difference(float angle1, float angle2);

#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#define max(a,b) a>b?a:b
#define min(a,b) a>b?b:a

// 整数转字符串
const char* int_to_str(int value);

// 无线发送两个整数
void sent_two_int(int a, int b);

#endif /* CODE_TOOL_H_ */
