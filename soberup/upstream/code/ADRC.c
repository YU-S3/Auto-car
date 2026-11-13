#include "ADRC.h"

// 抗积分饱和限幅
static float clamp_float(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// 初始化 LADRC
void LADRC_Init(LADRC_Struct *adrc, float b0, float wc, float wo, float h, float max_pwm, float min_pwm)
{
    // 防止除 0 错误
    if (b0 < 0.01f) b0 = 0.01f;

    adrc->b0 = b0;
    adrc->wc = wc;
    adrc->wo = wo;
    adrc->h  = h;
    adrc->max_pwm = max_pwm;
    adrc->min_pwm = min_pwm;

    // 自动计算 LESO 观测器增益
    adrc->beta1 = 2.0f * wo;
    adrc->beta2 = wo * wo;

    // 状态清零
    LADRC_Reset(adrc);
}

// 重置内部状态，防止停车时积分异常累积
void LADRC_Reset(LADRC_Struct *adrc)
{
    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->u  = 0.0f;
}

// LADRC 核心计算函数 (1ms 中断调用一次)
int LADRC_Calculate(LADRC_Struct *adrc, float target_speed, float actual_speed)
{
    float e;
    float u0;
    float u_raw;
    float z2_max;

    // 1. LESO (线性扩张状态观测器) - 实时估计转速和扰动
    e = adrc->z1 - actual_speed;

    // 状态更新：根据上一帧速度和扰动
    adrc->z1 += adrc->h * (adrc->z2 + adrc->b0 * adrc->u - adrc->beta1 * e);
    adrc->z2 -= adrc->h * adrc->beta2 * e;

    // 抗积分饱和：防止长时间大误差时 z2 积分爆炸 (设定为最大输出电压的1.5倍)
    z2_max = adrc->max_pwm * adrc->b0 * 1.5f;
    adrc->z2 = clamp_float(adrc->z2, -z2_max, z2_max);

    // 2. LSEF (线性状态误差反馈)
    u0 = adrc->wc * (target_speed - adrc->z1);

    // 扰动补偿 (用估计的 z2 抵消实际扰动)
    u_raw = (u0 - adrc->z2) / adrc->b0;

    // 3. 限幅并输出
    adrc->u = clamp_float(u_raw, adrc->min_pwm, adrc->max_pwm);

    return (int)(adrc->u);
}
