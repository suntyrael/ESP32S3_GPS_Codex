/*
 * ahrs.c - AHRS 适配层实现
 *
 * 算法本体来自 x-io Technologies Fusion v1.3.3（MIT）：
 *   FusionAhrsUpdate()   — 陀螺积分 + 加计/磁力计互补反馈
 *   FusionAhrsUpdateNoMagnetometer() — 磁干扰期间纯陀螺 + 加计
 *   FusionAhrsGetInternalStates()    — 读取本帧磁/加计是否被拒绝
 *   FusionAhrsGetFlags()             — 启动收敛 / 超量程 / 恢复标志
 *
 * 本文件不重复实现任何融合算法，只做坐标系与单位适配。
 */
#include "ahrs.h"
#include "config.h"
#include "FusionAhrs.h"
#include "FusionBias.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ahrs";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static FusionAhrs s_ahrs;
static FusionBias s_bias;
static ahrs_state_t s_state;
static bool s_initialised = false;
static bool s_initial_pose_done = false;

/* 静止窗口零偏快估（任意时刻，设备静止 2 秒即可锁定），之后由 FusionBias 长期精修。
 * 必须能覆盖"未做 IMU 校准"时较大的零偏，否则零偏会被积分成航向静差：
 * 静态航向误差 ≈ 零偏 / 融合增益（1 dps ≈ 2°，5 dps ≈ 10°，10 dps ≈ 20°）。 */
static bool s_boot_bias_done = false;
static int s_boot_bias_count = 0;
static float s_boot_bias_sum[3] = { 0, 0, 0 };
static float s_boot_acc_ref[3] = { 0, 0, 0 };
static float s_boot_mag_ref[3] = { 0, 0, 0 };

static float wrap360(float deg)
{
    while (deg < 0.0f) {
        deg += 360.0f;
    }
    while (deg >= 360.0f) {
        deg -= 360.0f;
    }
    return deg;
}

/*
 * 用静止时的加速度计（重力反作用力）与磁力计解析初始四元数，
 * 使上电瞬间航向即正确，无需等待 Fusion 3 秒启动收敛。
 *
 *   u = normalize(acc)            机体系天顶
 *   e = normalize(m x u)          机体系正东
 *   n = u x e                     机体系正北
 *   R = [e; n; u]（行向量）        机体系 -> ENU
 */
static bool solve_initial_quaternion(const float accel_g[3], const float mag_uT[3],
                                     FusionQuaternion *out)
{
    const float ax = accel_g[0], ay = accel_g[1], az = accel_g[2];
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 0.75f || an > 1.25f) {
        return false;               /* 上电时设备在运动，交给 Fusion 启动收敛 */
    }
    const float ux = ax / an, uy = ay / an, uz = az / an;

    const float mx = mag_uT[0], my = mag_uT[1], mz = mag_uT[2];
    const float ex = my * uz - mz * uy;
    const float ey = mz * ux - mx * uz;
    const float ez = mx * uy - my * ux;
    const float en = sqrtf(ex * ex + ey * ey + ez * ez);
    if (en < 1e-3f) {
        return false;
    }
    const float exn = ex / en, eyn = ey / en, ezn = ez / en;

    const float nx = uy * ezn - uz * eyn;
    const float ny = uz * exn - ux * ezn;
    const float nz = ux * eyn - uy * exn;

    /* R 的行为 e / n / u */
    const float r00 = exn, r01 = eyn, r02 = ezn;
    const float r10 = nx,  r11 = ny,  r12 = nz;
    const float r20 = ux,  r21 = uy,  r22 = uz;

    const float trace = r00 + r11 + r22;
    float w, x, y, z;
    if (trace > 0.0f) {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (r21 - r12) / s;
        y = (r02 - r20) / s;
        z = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const float s = sqrtf(1.0f + r00 - r11 - r22) * 2.0f;
        w = (r21 - r12) / s;
        x = 0.25f * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const float s = sqrtf(1.0f + r11 - r00 - r22) * 2.0f;
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25f * s;
        z = (r12 + r21) / s;
    } else {
        const float s = sqrtf(1.0f + r22 - r00 - r11) * 2.0f;
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25f * s;
    }

    const float qn = sqrtf(w * w + x * x + y * y + z * z);
    if (qn < 1e-6f) {
        return false;
    }
    out->element.w = w / qn;
    out->element.x = x / qn;
    out->element.y = y / qn;
    out->element.z = z / qn;
    return true;
}

void ahrs_init(void)
{
    FusionAhrsInitialise(&s_ahrs);

    const FusionAhrsSettings settings = {
        .sampleRate = 1000.0f / (float)SENSOR_LOOP_MS,
        .convention = FusionConventionEnu,     /* 地球系 X=东, Y=北, Z=天 */
        .gain = AHRS_GAIN,
        .gyroscopeRange = AHRS_GYRO_RANGE_DPS,
        .accelerationRejection = AHRS_ACCEL_REJECT_DEG,
        .magneticRejection = AHRS_MAG_REJECT_DEG,
        .rejectionTimeout = AHRS_REJECT_TIMEOUT_S,
    };
    FusionAhrsSetSettings(&s_ahrs, &settings);

    FusionBiasInitialise(&s_bias);
    const FusionBiasSettings bias_settings = {
        .sampleRate = 1000.0f / (float)SENSOR_LOOP_MS,
        .stationaryThreshold = AHRS_BIAS_STILL_DPS,
        .stationaryPeriod = AHRS_BIAS_STATIONARY_S,
    };
    FusionBiasSetSettings(&s_bias, &bias_settings);

    s_boot_bias_done = false;
    s_boot_bias_count = 0;
    s_boot_bias_sum[0] = s_boot_bias_sum[1] = s_boot_bias_sum[2] = 0.0f;

    memset(&s_state, 0, sizeof(s_state));
    s_state.quat[0] = 1.0f;
    s_initialised = true;
    s_initial_pose_done = false;
}

void ahrs_reset(void)
{
    if (!s_initialised) {
        return;
    }
    FusionAhrsRestart(&s_ahrs);
    memset(&s_state, 0, sizeof(s_state));
    s_state.quat[0] = 1.0f;
    s_initial_pose_done = false;
}

void ahrs_update(const float gyro_dps[3], const float accel_g[3],
                 const float mag_uT[3], bool mag_ok, float dt_s)
{
    if (!s_initialised) {
        return;
    }
    if (dt_s < 0.002f) {
        dt_s = 0.002f;
    }
    if (dt_s > 0.200f) {
        dt_s = 0.200f;
    }

    const FusionVector gyro = { .axis = { gyro_dps[0], gyro_dps[1], gyro_dps[2] } };
    const FusionVector accel = { .axis = { accel_g[0], accel_g[1], accel_g[2] } };
    const FusionVector mag = { .axis = { mag_uT[0], mag_uT[1], mag_uT[2] } };

    /* 首次有效数据：解析初始姿态，直接跳过 3 秒启动收敛 */
    if (!s_initial_pose_done && mag_ok) {
        FusionQuaternion q0;
        if (solve_initial_quaternion(accel_g, mag_uT, &q0)) {
            FusionAhrsSetQuaternion(&s_ahrs, q0);
            /* 注意：不调用 FusionAhrsSkipStartup()。
             * 保留 Fusion 的 3 秒启动收敛期（高增益、且禁用加计/磁拒绝），
             * 即使初始四元数受上电晃动或磁干扰影响，也能在 3 秒内被传感器纠正，
             * 避免"跳过启动 + 磁拒绝"导致航向锁死在错误值。 */
            s_initial_pose_done = true;
        }
    }

    /* 静止窗口零偏快估：
     * 判据 = 陀螺各轴 < 15 dps（覆盖未校准零偏）
     *      + 窗口内加速度/磁场方向变化 < 3°（排除缓慢匀速转动）
     *      + 连续 40 帧（20 Hz × 2 s）。 */
    if (!s_boot_bias_done) {
        const float an = sqrtf(accel_g[0] * accel_g[0] +
                               accel_g[1] * accel_g[1] +
                               accel_g[2] * accel_g[2]);
        const float mn = sqrtf(mag_uT[0] * mag_uT[0] +
                               mag_uT[1] * mag_uT[1] +
                               mag_uT[2] * mag_uT[2]);
        bool still = mag_ok && an > 0.5f && an < 1.5f && mn > 1.0f &&
                     fabsf(gyro.axis.x) < AHRS_BOOT_BIAS_STILL_DPS &&
                     fabsf(gyro.axis.y) < AHRS_BOOT_BIAS_STILL_DPS &&
                     fabsf(gyro.axis.z) < AHRS_BOOT_BIAS_STILL_DPS;

        if (still && s_boot_bias_count > 0) {
            const float ax = accel_g[0] / an, ay = accel_g[1] / an, az = accel_g[2] / an;
            const float mx = mag_uT[0] / mn, my = mag_uT[1] / mn, mz = mag_uT[2] / mn;
            const float acc_dot = ax * s_boot_acc_ref[0] + ay * s_boot_acc_ref[1] + az * s_boot_acc_ref[2];
            const float mag_dot = mx * s_boot_mag_ref[0] + my * s_boot_mag_ref[1] + mz * s_boot_mag_ref[2];
            const float cos_lim = cosf(AHRS_BOOT_BIAS_SPAN_DEG * (float)M_PI / 180.0f);
            if (acc_dot < cos_lim || mag_dot < cos_lim) {
                still = false;
            }
        }

        if (!still) {
            s_boot_bias_count = 0;
            s_boot_bias_sum[0] = s_boot_bias_sum[1] = s_boot_bias_sum[2] = 0.0f;
        } else {
            if (s_boot_bias_count == 0) {
                s_boot_acc_ref[0] = accel_g[0] / an;
                s_boot_acc_ref[1] = accel_g[1] / an;
                s_boot_acc_ref[2] = accel_g[2] / an;
                s_boot_mag_ref[0] = mag_uT[0] / mn;
                s_boot_mag_ref[1] = mag_uT[1] / mn;
                s_boot_mag_ref[2] = mag_uT[2] / mn;
            }
            s_boot_bias_sum[0] += gyro.axis.x;
            s_boot_bias_sum[1] += gyro.axis.y;
            s_boot_bias_sum[2] += gyro.axis.z;
            s_boot_bias_count++;
            if (s_boot_bias_count >= AHRS_BOOT_BIAS_SAMPLES) {
                const FusionVector offset = {
                    .axis = {
                        s_boot_bias_sum[0] / (float)AHRS_BOOT_BIAS_SAMPLES,
                        s_boot_bias_sum[1] / (float)AHRS_BOOT_BIAS_SAMPLES,
                        s_boot_bias_sum[2] / (float)AHRS_BOOT_BIAS_SAMPLES,
                    }
                };
                FusionBiasSetOffset(&s_bias, offset);
                s_boot_bias_done = true;
                ESP_LOGI(TAG, "Static gyro bias: %+.2f %+.2f %+.2f dps",
                         (double)offset.axis.x, (double)offset.axis.y, (double)offset.axis.z);
            }
        }
    }

    /* FusionBias：静止时用高通滤波长期修正残余零偏（含温漂） */
    const FusionVector gyro_corrected = FusionBiasUpdate(&s_bias, gyro);

    FusionAhrsSetSamplePeriod(&s_ahrs, dt_s);
    if (mag_ok) {
        FusionAhrsUpdate(&s_ahrs, gyro_corrected, accel, mag);
    } else {
        FusionAhrsUpdateNoMagnetometer(&s_ahrs, gyro_corrected, accel);
    }
    s_state.updates++;

    const FusionQuaternion q = FusionAhrsGetQuaternion(&s_ahrs);
    const FusionAhrsFlags flags = FusionAhrsGetFlags(&s_ahrs);
    const FusionAhrsInternalStates internal = FusionAhrsGetInternalStates(&s_ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(q);

    const float w = q.element.w, x = q.element.x;
    const float y = q.element.y, z = q.element.z;

    /* 机体系天顶分量 = R 第三行 */
    const float ux = 2.0f * (x * z - w * y);
    const float uy = 2.0f * (y * z + w * x);
    const float uz = 1.0f - 2.0f * (x * x + y * y);

    /* 机身 +Y（顶部）与 +Z（屏幕法线）在地球系的水平分量 = R 第 2/3 列 */
    const float yx = 2.0f * (x * y - w * z);
    const float yy = 1.0f - 2.0f * (x * x + z * z);
    const float zx = 2.0f * (x * z + w * y);
    const float zy = 2.0f * (y * z - w * x);

    /* 平放取机身顶部；竖直立起时顶部进入奇异点，平滑切换到屏幕背面视线 */
    float w_fwd = uz > 0.0f ? uz : 0.0f;
    float w_back = uy > 0.0f ? uy : 0.0f;
    if (w_fwd + w_back < 1e-3f) {          /* 屏幕朝下等极端姿态兜底 */
        w_fwd = fabsf(uz);
        w_back = fabsf(uy);
    }

    const float fx = w_fwd * yx - w_back * zx;   /* 东向分量 */
    const float fy = w_fwd * yy - w_back * zy;   /* 北向分量 */

    const FusionVector bias_offset = FusionBiasGetOffset(&s_bias);
    s_state.gyro_bias_dps[0] = bias_offset.axis.x;
    s_state.gyro_bias_dps[1] = bias_offset.axis.y;
    s_state.gyro_bias_dps[2] = bias_offset.axis.z;

    s_state.quat[0] = w;
    s_state.quat[1] = x;
    s_state.quat[2] = y;
    s_state.quat[3] = z;
    s_state.heading_deg = wrap360(atan2f(fx, fy) * (180.0f / (float)M_PI));
    /* Fusion 的 ZYX 欧拉角：绕 X 为"顶部抬升"，绕 Y 为"左右侧倾" */
    s_state.pitch_deg = euler.angle.roll;
    s_state.roll_deg = euler.angle.pitch;
    s_state.startup = flags.startup;
    s_state.mag_ignored = mag_ok ? internal.magnetometerIgnored : true;
    s_state.accel_ignored = internal.accelerometerIgnored;
    s_state.valid = s_initial_pose_done || !flags.startup;
}

void ahrs_get_state(ahrs_state_t *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_state;
}
