#ifndef CODE_ADRC_H_
#define CODE_ADRC_H_

#include "zf_common_headfile.h"

// ==========================================
// 一阶 LADRC 结构体定义
// ==========================================
typedef struct {
    // 1. 核心可调参数
    float b0;           // 系统增益 (PWM到速度的放大系数)
    float wc;           // 控制器带宽 (决定响应速度，类比PID的P)
    float wo;           // 观测器带宽 (决定扰动估计速度，通常设为 wc 的 3~5 倍)
    float h;            // 控制周期 (单位秒，例如 1ms = 0.001)

    // 2. 输出限幅
    float max_pwm;      // 最大输出PWM
    float min_pwm;      // 最小输出PWM (负值)

    // 3. 内部自动计算的观测器增益
    float beta1;
    float beta2;

    // 4. 状态变量
    float z1;           // 观测器估计的平滑转速
    float z2;           // 观测器估计的总扰动 (电压/负载/摩擦力等)
    float u;            // 上次输出的控制量 (PWM)
} LADRC_Struct;

// ==========================================
// 函数声明
// ==========================================
void LADRC_Init(LADRC_Struct *adrc, float b0, float wc, float wo, float h, float max_pwm, float min_pwm);
int LADRC_Calculate(LADRC_Struct *adrc, float target_speed, float actual_speed);
void LADRC_Reset(LADRC_Struct *adrc);

#endif /* CODE_ADRC_H_ */
