#include "MahonyAHRS.h"
#include "zf_device_imu660rc.h"
#include "zf_driver_pit.h"
#include "zf_driver_flash.h"
#include "menu.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------
float twoKi;                                  // 2 * integral gain (Ki)
float q0, q1, q2, q3;                         // quaternion of sensor frame relative to auxiliary frame
float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
float invSampleFreq;
float roll_mahony, pitch_mahony, yaw_mahony;
float GZ;

#define IMU_SAMPLE_PERIOD_S (0.002f)

static volatile float continuous_yaw_deg = 0.0f;
static volatile float gyro_z_peak_abs = 0.0f;
static float previous_gyro_z_dps = 0.0f;
static uint8 continuous_yaw_initialized = 0U;

imu_param_t imu_data;
euler_param_t eulerAngle;

// 静态零漂变量
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

//-------------------------------------------------------------------------------------------
#define twoKpDef    (2.0f * 5.0f)   // 2 * proportional gain
#define twoKiDef    (2.0f * 0.0f)   // 2 * integral gain (可根据需要改为 0.005f 来启用积分项)

// 快速平方根倒数
float Mahony_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ======================= 陀螺仪上电去零偏校准 =======================
void Mahony_CalibrateGyro(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t sample_count = 500; // 采集500次取平均, 约2.5s

    for(uint16_t i = 0; i < sample_count; i++)
    {
        imu660rc_get_gyro(); // 读取 IMU660RC 陀螺仪原始数据

        // 累加转换为角速度(DPS)的值
        sum_x += imu660rc_gyro_transition(imu660rc_gyro_x);
        sum_y += imu660rc_gyro_transition(imu660rc_gyro_y);
        sum_z += imu660rc_gyro_transition(imu660rc_gyro_z);

        Delay(5); // 每次读取间隔5ms, 总校准时间约1秒
    }

    // 计算平均零漂值 (单位: DPS)
    gyro_offset_x = sum_x / sample_count;
    gyro_offset_y = sum_y / sample_count;
    gyro_offset_z = sum_z / sample_count;
}

// ======================= Flash 零漂存取 =======================
// 零漂已合并到统一参数页 PARAM_PAGE, 偏移 O_GYRO_X/Y/Z/FLAG
// 返回 1=加载成功, 0=无有效校准数据
uint8 Gyro_LoadOffsets(void)
{
    float buf[PARAM_WORD_COUNT];
    memset(buf, 0, sizeof(buf));

    if (flash_check(0, PARAM_PAGE) != 1)
        return 0;

    flash_read_page(0, PARAM_PAGE, (uint32 *)buf, PARAM_WORD_COUNT);

    if (buf[O_GYRO_FLAG] != 1.0f)
        return 0;

    if (isnan(buf[O_GYRO_X]) || isnan(buf[O_GYRO_Y]) || isnan(buf[O_GYRO_Z]))
        return 0;

    gyro_offset_x = buf[O_GYRO_X];
    gyro_offset_y = buf[O_GYRO_Y];
    gyro_offset_z = buf[O_GYRO_Z];
    return 1;
}

// 保存当前零漂到统一参数页 PARAM_PAGE
void Gyro_SaveOffsets(void)
{
    float buf[PARAM_WORD_COUNT];
    // 先读出整页 (保留其他参数), 修改陀螺字段, 再写回
    if (flash_check(0, PARAM_PAGE) == 1) {
        flash_read_page(0, PARAM_PAGE, (uint32 *)buf, PARAM_WORD_COUNT);
    } else {
        memset(buf, 0, sizeof(buf));
    }

    buf[O_GYRO_X] = gyro_offset_x;
    buf[O_GYRO_Y] = gyro_offset_y;
    buf[O_GYRO_Z] = gyro_offset_z;
    buf[O_GYRO_FLAG] = 1.0f;  // 校准标志

    flash_write_page(0, PARAM_PAGE, (const uint32 *)buf, PARAM_WORD_COUNT);
}

// 陀螺仪低通滤波系数: 消除负压风扇振动对姿态解算的干扰
// alpha 越小滤波越强, 0.0~1.0, 0.3 为推荐起点
#define GYRO_LPF_ALPHA  0.2

static void IMU_UpdateContinuousYaw(float gyro_z_dps)
{
    float gyro_z_abs = fabsf(gyro_z_dps);

    if (gyro_z_abs > gyro_z_peak_abs)
    {
        gyro_z_peak_abs = gyro_z_abs;
    }

    if (!continuous_yaw_initialized)
    {
        previous_gyro_z_dps = gyro_z_dps;
        continuous_yaw_initialized = 1U;
        return;
    }

    continuous_yaw_deg +=
        0.5f * (previous_gyro_z_dps + gyro_z_dps) * IMU_SAMPLE_PERIOD_S;
    previous_gyro_z_dps = gyro_z_dps;
}

// 读取IMU传感器数据并转换单位
void IMU_GetValue()
{
    static float gyro_x_filt = 0.0f, gyro_y_filt = 0.0f, gyro_z_filt = 0.0f;
    static uint8_t gyro_filt_inited = 0;

    // 1. 读取 IMU660RC 原始数据
    imu660rc_get_gyro();
    imu660rc_get_acc();

    // 2. 加速度计数据转换
    imu_data.acc_x = imu660rc_acc_transition(imu660rc_acc_x);
    imu_data.acc_y = imu660rc_acc_transition(imu660rc_acc_y);
    imu_data.acc_z = -imu660rc_acc_transition(imu660rc_acc_z);

    // 3. 陀螺仪数据转换并去零漂
    float gyro_x_dps = imu660rc_gyro_transition(imu660rc_gyro_x) - gyro_offset_x;
    float gyro_y_dps = imu660rc_gyro_transition(imu660rc_gyro_y) - gyro_offset_y;
    float gyro_z_dps = imu660rc_gyro_transition(imu660rc_gyro_z) - gyro_offset_z;

    // 4. 低通滤波消除负压振动高频噪声
    if (!gyro_filt_inited) {
        gyro_x_filt = gyro_x_dps;
        gyro_y_filt = gyro_y_dps;
        gyro_z_filt = gyro_z_dps;
        gyro_filt_inited = 1;
    } else {
        gyro_x_filt = GYRO_LPF_ALPHA * gyro_x_dps + (1.0f - GYRO_LPF_ALPHA) * gyro_x_filt;
        gyro_y_filt = GYRO_LPF_ALPHA * gyro_y_dps + (1.0f - GYRO_LPF_ALPHA) * gyro_y_filt;
        gyro_z_filt = GYRO_LPF_ALPHA * gyro_z_dps + (1.0f - GYRO_LPF_ALPHA) * gyro_z_filt;
    }

    // 5. 角速度(DPS)转换为 Mahony 需要的 rad/s (弧度每秒)
    imu_data.gyro_x = gyro_x_filt * (M_PI / 180.0f);
    imu_data.gyro_y = gyro_y_filt * (M_PI / 180.0f);
    imu_data.gyro_z = gyro_z_filt * (M_PI / 180.0f);

    // 转向阻尼继续使用当前软件滤波结果，单位保持为 °/s。
    GZ = gyro_z_filt;

    // 三叉转角与峰值诊断只使用去零偏后的Z轴角速度，不接受加速度计反馈。
    IMU_UpdateContinuousYaw(gyro_z_dps);
}

// IMU姿态更新互补滤波算法
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc, qd;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex  * invSampleFreq;
            integralFBy += twoKi * halfey  * invSampleFreq;
            integralFBz += twoKi * halfez  * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    gx *= (0.5f  * invSampleFreq);
    gy *= (0.5f  * invSampleFreq);
    gz *= (0.5f  * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;
    qd = q3;

    q0 += (-qb * gx - qc * gy - qd * gz);
    q1 += (qa * gx + qc * gz - qd * gy);
    q2 += (qa * gy - qb * gz + qd * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void QuaternionsToEulerAngle()
{
    roll_mahony = asinf(-2.0f * (q1*q3 - q0*q2));
    eulerAngle.roll = 57.29578f * roll_mahony;

    pitch_mahony = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    eulerAngle.pitch = 57.29578f * pitch_mahony;

    yaw_mahony = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    eulerAngle.yaw = 57.29578f * yaw_mahony;
}

void Mahony_GetAngles()
{
    IMU_GetValue();
    MahonyAHRSupdateIMU(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
    QuaternionsToEulerAngle();
}

void Get_InitAngle(float ax, float ay, float az)
{
    float recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float roll_init = atan2f(ay, az);
    float pitch_init = atan2f(-ax, sqrtf(ay*ay + az*az));

    float cr = cosf(roll_init * 0.5f);
    float sr = sinf(roll_init * 0.5f);
    float cp = cosf(pitch_init * 0.5f);
    float sp = sinf(pitch_init * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    recipNorm = Mahony_invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_Init(float sampleFrequency)
{
    // 如果 IMU660RC 未在外部初始化, 则应在此硬件初始化
    imu660rc_init(IMU660RC_QUARTERNION_DISABLE);

    twoKi = twoKiDef;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    continuous_yaw_deg = 0.0f;
    previous_gyro_z_dps = 0.0f;
    continuous_yaw_initialized = 0U;
    IMU_ResetRunDiagnostics();
    invSampleFreq = 1.0f / sampleFrequency;

    Delay(100); // 稳定等待

    // 1. 优先从Flash加载零漂, 若无效则做静态校准
    if (!Gyro_LoadOffsets())
    {
        Mahony_CalibrateGyro();
    }

    // 2. 使用加速度计获取一个初始角度
    imu660rc_get_acc();
    float init_ax = imu660rc_acc_transition(imu660rc_acc_x);
    float init_ay = imu660rc_acc_transition(imu660rc_acc_y);
    float init_az = -imu660rc_acc_transition(imu660rc_acc_z);

    Get_InitAngle(init_ax, init_ay, init_az);
}

float get_yaw(){
    return eulerAngle.yaw;
}

float get_continuous_yaw(void)
{
    return continuous_yaw_deg;
}

float get_gyro_z_peak_abs(void)
{
    return gyro_z_peak_abs;
}

void IMU_ResetRunDiagnostics(void)
{
    gyro_z_peak_abs = 0.0f;
}

float get_pitch(){
    return eulerAngle.pitch;
}

void imu660rc_init_main(){
    Mahony_Init(500);
    pit_ms_init(CCU60_CH1,2);
}
