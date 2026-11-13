#ifndef __MahonyAHRS_H
#define __MahonyAHRS_H

// 本文件为 IMU660RC 的头文件
#include "zf_device_imu660rc.h"

// 延时应根据系统的实际延时函数来确定
#define Delay(time)     system_delay_ms(time)

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

// 建议将原来的 icm_param_t 重命名为通用名称 imu_param_t
typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_param_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} euler_param_t;

extern imu_param_t imu_data;
extern euler_param_t eulerAngle;
extern float GZ;

// 暴露静态校准的陀螺偏值，供其他模块读取
extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;

// 陀螺零漂已合并到统一参数页 PARAM_PAGE, 偏移 O_GYRO_X/Y/Z/FLAG
uint8 Gyro_LoadOffsets(void);   // 从Flash(PARAM_PAGE)加载零漂, 返回1成功/0失败
void Gyro_SaveOffsets(void);    // 保存当前零漂到Flash(PARAM_PAGE)

void Mahony_Init(float sampleFrequency);
void Mahony_CalibrateGyro(void); // 重新进行静态校准陀螺零漂
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_GetAngles(void);
float get_yaw(void);
float get_continuous_yaw(void);
float get_gyro_z_peak_abs(void);
void IMU_ResetRunDiagnostics(void);
void imu660rc_init_main();
float get_pitch(void);

#endif
