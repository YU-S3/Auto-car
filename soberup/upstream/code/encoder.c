/**************************************************************
 * 文件名:  encoder.c
 * 创建时间: 2026年3月6日 下午3:20:21
 * 说明:
 **************************************************************/
#include "encoder.h"
#include "zf_driver_encoder.h"

void Encoder_Init(void){
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
}


#include <stdlib.h> // 需要用到 abs() 计算绝对值

// 编码器速度突变阈值 (太小会导致误检测为数据异常)
// 当前控制器每5ms采样一次，5ms内速度变化绝对不可能超过300。超过了肯定是硬件读错了。
#define ENCODER_SPIKE_THRESHOLD 300

/**************************************************************
 * 获取左轮当前速度 (限幅防突变 + 低通滤波)
 **************************************************************/
int Get_Encoder_L() {
    static float filtered_L = 0;  // 滤波输出值
    static int last_raw = 0;      // 记录上一次可靠原始值

    // 1. 读取原始值并清零
    int raw = (int)encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);

    // 2. 突变滤波: 比较本次与上次可靠数据的差值
    //    如果差值异常(超过阈值)，说明读到了尖峰噪声
    if (abs(raw - last_raw) > ENCODER_SPIKE_THRESHOLD) {
        raw = last_raw;  // 本次数据认定为野点，直接丢弃并沿用上一次的值
    } else {
        last_raw = raw;  // 正常值，更新历史记录
    }

    // 3. 应用低通滤波公式 (此时喂进去的 raw 已经是干净安全的数据)
    filtered_L = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filtered_L;

    // 4. 返回整数结果（四舍五入）
    return (int)(filtered_L + 0.5f);
}

/**************************************************************
 * 获取右轮当前速度 (限幅防突变 + 低通滤波)
 **************************************************************/
int Get_Encoder_R() {
    static float filtered_R = 0;  // 滤波输出值
    static int last_raw = 0;      // 记录上一次可靠原始值

    // 1. 读取原始值并清零
    int raw = (int)encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);

    // 2. 突变滤波
    if (abs(raw - last_raw) > ENCODER_SPIKE_THRESHOLD) {
        raw = last_raw;  // 丢弃异常
    } else {
        last_raw = raw;  // 更新记录
    }

    // 3. 应用低通滤波公式
    filtered_R = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filtered_R;

    // 4. 返回整数结果（四舍五入）
    return (int)(filtered_R + 0.5f);
}
