/*
 * sensors.h - 汇聚层（唯一写者）
 * 所有硬件状态由 sensor_task 周期调用 sensors_update() 写入，
 * 其他任务通过 sensors_get_state() 获取互斥保护下的按值拷贝快照（D-02）。
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool valid;
    uint8_t fails;              /* 连续读取失败次数（达到 SENSOR_FAIL_LIMIT 才置 invalid） */
    float accel_mg[3];      /* 总加速度（含重力）mg */
    float lin_mg[3];        /* 线性加速度（重力估计分离后）mg */
    float gyro_mdps[3];     /* 角速度 mdps */
    float quat[4];          /* AHRS 四元数 w,x,y,z（机体系 -> ENU） */
    float roll_deg;         /* 机身左右侧倾（右侧下沉为正） */
    float pitch_deg;        /* 机身顶部抬升（平放 0°，竖直立起 +90°） */
    float ahrs_heading_deg; /* AHRS 解算磁北航向 0~360° */
    float gyro_bias_dps[3]; /* AHRS 运行时陀螺零偏估计（dps） */
    bool ahrs_valid;        /* 四元数姿态已收敛可用 */
    bool mag_ignored;       /* 本帧磁力计被 AHRS 拒绝（磁干扰） */
    bool accel_ignored;     /* 本帧加速度计被 AHRS 拒绝（线性加速/振动） */
    float temp_c;
} sensors_imu_t;

typedef struct {
    bool valid;
    uint8_t fails;
    float mag_mgauss[3];
    float heading_deg;          /* 电子罗盘解算磁航向 (0.0° ~ 359.9°) */
    float heading_static_deg;   /* 静态 3D 倾角补偿航向（用于与 AHRS 对比诊断） */
    float heading_true_deg;     /* GPS地磁偏角补偿后真北航向 (0.0° ~ 359.9°) */
    float declination_deg;      /* 当前地磁偏角 (度, 东偏为正, 西偏为负) */
    bool declination_valid;     /* 是否已获得有效 GPS 地磁偏角补偿 */
    bool disturbed;             /* 磁干扰：AHRS 已拒绝本帧磁力计 */
    float temp_c;
} sensors_mag_t;

typedef struct {
    bool valid;
    uint8_t fails;
    float temp_c;
    float pressure_hpa;
    float altitude_m;
} sensors_baro_t;

typedef struct {
    bool valid;
    uint8_t fails;
    float voltage_v;
    uint16_t adc_mv;
    int raw_count;
    uint8_t percent;
    bool saturated;
    bool charging;
} sensors_battery_t;

typedef struct {
    sensors_imu_t imu;
    sensors_mag_t mag;
    sensors_baro_t baro;
    sensors_battery_t battery;
    /* GNSS 字段在阶段 3 扩展 */
} sensors_state_t;

/**
 * @brief 初始化：I2C 总线 + 三个传感器驱动 + 电池
 * @return ESP_OK 全部关键器件初始化成功（单器件失败仍返回 ESP_OK，仅 valid=false）
 */
esp_err_t sensors_init(void);

/** @brief 周期采样并更新内部状态（仅 sensor_task 调用） */
esp_err_t sensors_update(void);

/** @brief 获取状态快照（按值拷贝，线程安全） */
void sensors_get_state(sensors_state_t *out);

/** @brief 各通道 valid 汇总：返回 true 表示全部就绪 */
bool sensors_all_ready(void);

/** @brief 通过已知绝对高度（如 GNSS 高度）校准气压计基准 */
void sensors_calibrate_altitude(float known_alt_m);

/** @brief 计算指定经纬度的地磁偏角（度，东偏为正，西偏为负） */
float sensors_calc_magnetic_declination(double lat_deg, double lon_deg);

/** @brief 根据 GPS 定位经纬度自动更新地磁偏角补偿 */
void sensors_update_magnetic_declination(double lat_deg, double lon_deg);

/* ==================== 真实传感器物理校准引擎 ==================== */
typedef enum {
    SENSORS_CALIB_MODE_IDLE = 0,
    SENSORS_CALIB_MODE_IMU,     /* 真实 IMU 水平静止采样 */
    SENSORS_CALIB_MODE_MAG,     /* 真实地磁 8 字三维空间覆盖采样 */
    SENSORS_CALIB_MODE_DONE,    /* 采样达标完成 */
} sensors_calib_mode_t;

typedef struct {
    sensors_calib_mode_t mode;
    int imu_pct;            /* 真实 IMU 静止有效采样进度 (0~100) */
    bool imu_is_still;      /* 当前是否处于真实水平静止状态 */
    int mag_pct;            /* 真实地磁 3D 姿态覆盖进度 (0~100) */
    bool mag_motion_ok;     /* 是否检测到有效的三维动态旋转 */
    bool imu_ready;         /* IMU 校准是否已达 100% 收敛 */
    bool mag_ready;         /* 地磁校准是否已达 100% 收敛 */
    bool timeout;           /* 30秒超时标志（超时未完成需重新校准） */
} sensors_calib_live_status_t;

/** @brief 开始指定模式的真实物理校准采样 */
void sensors_calibration_start(sensors_calib_mode_t mode);

/** @brief 取消/中止当前校准流程 */
void sensors_calibration_cancel(void);

/** @brief 获取当前真实校准计算状态快照（线程安全，供 UI 轮询显示） */
void sensors_calibration_get_status(sensors_calib_live_status_t *out);

/** @brief 保存当前校准结果到 NVS Flash 并立即生效 */
void sensors_calibration_save(void);
