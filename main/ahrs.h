/*
 * ahrs.h - 姿态航向参考系统（AHRS）适配层
 *
 * 底层算法使用 x-io Technologies Fusion 库（MIT，vendored 于 components/fusion/），
 * 本模块只负责：机体系/地球系约定转换、单位换算、启动初始姿态解析、
 * 以及把四元数输出转换为本项目 UI/业务可直接使用的航向与倾角。
 *
 * 坐标系约定（与 sensors.h 一致）：
 *   机体系：X = 屏幕向右, Y = 机身顶部, Z = 屏幕法线向外（平放朝上）
 *   地球系：ENU（X = 东, Y = 北, Z = 天）
 *   因此设备水平平放且机身顶部朝北时，四元数为单位四元数。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float quat[4];        /* 四元数 w, x, y, z（机体系 -> ENU） */
    float roll_deg;       /* 机身左右侧倾：右侧下沉为正，范围约 ±90° */
    float pitch_deg;      /* 机身顶部抬升：平放 0°，竖直立起 +90° */
    float heading_deg;    /* 磁北航向 0~360°（平放取机身顶部，竖立取屏幕背面视线） */
    float gyro_bias_dps[3]; /* 当前陀螺零偏估计（dps，FusionBias + 上电快估） */
    bool  valid;          /* 姿态已收敛，可对外使用 */
    bool  startup;        /* Fusion 仍处于启动收敛期 */
    bool  mag_ignored;    /* 本帧磁力计被拒绝（磁干扰/异常） */
    bool  accel_ignored;  /* 本帧加速度计被拒绝（线性加速/振动） */
    uint32_t updates;     /* 累计更新次数（验证用） */
} ahrs_state_t;

/** @brief 初始化（上电调用一次） */
void ahrs_init(void);

/** @brief 复位姿态到单位四元数并重新进入启动收敛 */
void ahrs_reset(void);

/**
 * @brief 姿态更新
 * @param gyro_dps  角速度 dps（已校准）
 * @param accel_g   加速度 g（已校准，含重力反作用力）
 * @param mag_uT    磁场 uT（已校准 + 已映射到机体系）
 * @param mag_ok    false 时走纯陀螺 + 加速度计通道（磁干扰期间）
 * @param dt_s      采样间隔（秒）
 */
void ahrs_update(const float gyro_dps[3], const float accel_g[3],
                 const float mag_uT[3], bool mag_ok, float dt_s);

/** @brief 获取最新姿态快照（按值拷贝） */
void ahrs_get_state(ahrs_state_t *out);
