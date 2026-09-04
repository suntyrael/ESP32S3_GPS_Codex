/*
 * config.h - 全项目唯一配置来源（引脚 / 参数）
 * 约束：D-12 零魔数——所有可调参数收敛于此，禁止散落
 */
#pragma once

#include <stdint.h>
#include "hal/adc_types.h"   /* ADC_UNIT_2/ADC_CHANNEL_1/ADC_ATTEN_DB_11/ADC_BITWIDTH_12 */

/* ==================== 固件信息 ==================== */
#define FW_VERSION_MAJOR        0
#define FW_VERSION_MINOR        3
#define FW_VERSION_PATCH        0
#define FW_VERSION_STR          "V0.3.0"

/* ==================== 引脚分配（原理图已核实） ==================== */
/* I2C0 */
#define PIN_I2C_SCL             39
#define PIN_I2C_SDA             40
#define I2C_BUS_CLK_HZ          500000      /* 500 kHz：用户要求降速验证 MAG 稳定性 */
#define I2C_BUS_GLITCH_CNT      7

/* LCD（阶段 2 启用） */
#define PIN_LCD_SCK             5
#define PIN_LCD_MOSI            8
#define PIN_LCD_CS              7
#define PIN_LCD_DC              6
#define PIN_LCD_RST             4
#define PIN_LCD_BL              9
#define LCD_SPI_HOST            SPI3_HOST
#define LCD_SPI_CLK_HZ          80000000   /* SPI 时钟 80 MHz */
#define LCD_H_RES               240
#define LCD_V_RES               320
#define LCD_INVERT_COLOR        1           /* ST7789 通常需反转颜色（1=反转） */
#define LCD_MIRROR_X            1           /* 旋转 180°：X 镜像 */
#define LCD_MIRROR_Y            1           /* 旋转 180°：Y 镜像 */
#define LCD_SWAP_XY             0
#define LCD_SET_GAP             0           /* 模组有偏移时置 1 并设 GAP_X/Y */
#define LCD_GAP_X               0
#define LCD_GAP_Y               0
#define LCD_BL_PWM_HZ           2000        /* 背光 PWM 2 kHz */
#define LCD_BL_DEFAULT_PERCENT  50          /* 默认背光 50%（README §3.2） */
#define LCD_POWER_ON_DELAY_MS   20          /* 首帧写入显存后，等待液晶稳态再开背光的延时（ms） */

/* GNSS UART1（阶段 3 启用） */
#define PIN_GNSS_TX             17
#define PIN_GNSS_RX             18
#define PIN_GNSS_LDO_EN         14

/* 传感器中断（当前未用，禁止占用） */
#define PIN_ACCGYRO_INT         41
#define PIN_MAG_INT             42
#define PIN_PRESS_INT           13

/* ==================== 编码器 / 按键（阶段 4 启用） ==================== */
#define PIN_ENC_A               1
#define PIN_ENC_B               3
#define PIN_KEY_MAIN            2
#define ENC_WINDOW_MS           300         /* 编码器脉冲防抖与未转动清零超时（ms） */
#define KEY_DEBOUNCE_MS         20          /* 按键消抖时间（ms） */
#define KEY_LONG_MIN_MS         800         /* 长按判定阈值（ms），达到时立即触发底层操作无需等待松手 */
#define KEY_DOUBLE_GAP_MS       400         /* 双击判定最大间隔（ms） */

/* ==================== P-Box（README §4.4） ==================== */
#define PBOX_ACC_THRESHOLD_G    0.15f       /* IMU X 轴线性加速度启动阈值（0.10~0.30 可调） */
#define PBOX_START_SPEED_KMH    1.0f        /* 启动前静止速度阈值 */
#define PBOX_TARGETS_KMH        { 60.0f, 100.0f }   /* 目标区间（km/h） */

/* 电源 */
#define PIN_BAT_ADC             12          /* ADC2_CH1 */
#define PIN_CHG_SAT             21          /* TP4054 开漏：低=充电中 */
#define CHG_SAT_ACTIVE_LOW      1           /* 1=低电平表示充电中 */

/* USB / 下载键 */
#define PIN_USB_DP              20
#define PIN_USB_DN              19
#define PIN_DL_KEY              0

/* 空闲引脚（禁止分配外设） */
#define PIN_GPIO10              10
#define PIN_GPIO11              11          /* WATCHDOG，暂不开发 */
#define PIN_GPIO15              15
#define PIN_GPIO16              16

/* ==================== 电池 ==================== */
#define BAT_ADC_UNIT            ADC_UNIT_2
#define BAT_ADC_CHANNEL         ADC_CHANNEL_1
#define BAT_ADC_ATTEN           ADC_ATTEN_DB_12   /* v6 中 11dB 已改名 DB_12 */
#define BAT_ADC_BITWIDTH        ADC_BITWIDTH_12
#define BAT_SATURATION_MV       3050        /* 校准后饱和阈值（~3.1V 量程上限） */
#define BAT_AVG_SAMPLES         8           /* 单次采样平均次数（降 ADC 抖动，日志波动 ±3% → <1%） */
#define BAT_PCT_EMA_ALPHA       0.0025f     /* 百分比 20s 平滑（50ms 采样 → 时间常数 1/alpha*dt ≈ 20s） */
#define BAT_VOLT_FULL_MV        4200        /* 满电电压 */
#define BAT_VOLT_EMPTY_MV       3000        /* 空电电压 */
#define BAT_DIVIDER_RATIO       2.0f        /* 分压比：电池电压 = ADC电压×比值。
                                              * 外部分压电阻 1:1（R1=R2）→ 引脚=电池/2 → 比值 2.0
                                              * 实测：电池 4.25V / 引脚 2.0976V = 2.03 ≈ 2.0 ✓ */

/* ==================== IMU（LSM6DSR） ==================== */
#define IMU_I2C_ADDR_A          0x6A
#define IMU_I2C_ADDR_B          0x6B
#define IMU_WHO_AM_I_1          0x6B
#define IMU_WHO_AM_I_2          0x6A
#define IMU_ODR_HZ              104         /* 加速度/陀螺 ODR */
#define IMU_ACCEL_FS_G          2           /* ±2g */
#define IMU_GYRO_FS_DPS         2000        /* ±2000dps */
/* 轴向映射（竖置：X=前进/Y=竖直/Z=屏幕法线；符号待实测） */
#define IMU_AXIS_X_SIGN         1
#define IMU_AXIS_Y_SIGN         1
#define IMU_AXIS_Z_SIGN         1

/* ==================== 磁力计（LIS2MDL） ==================== */
#define MAG_I2C_ADDR            0x1E
#define MAG_WHO_AM_I            0x40
#define MAG_CFG_REG_A           0x80        /* COMP_TEMP_EN=1(必置1，DS p21 脚注)+连续模式10Hz(MD=00,ODR=00,HR) */
#define MAG_CFG_REG_B           0x12        /* IF_ADD_INC(0x02) + BDU(0x10)：多字节自增 + 数据锁存（防异步读字节撕裂） */
#define LIS2MDL_READ_RETRY      3           /* 单帧读取重试次数 */
#define LIS2MDL_FAIL_REINIT     3           /* 连续失败 N 帧后触发总线恢复+重初始化 */
/* 磁力计机身坐标系映射（U10在PCB上逆时针旋转90°贴装：X_dev = +OUTY, Y_dev = -OUTX, Z_dev = -OUTZ） */
#define MAG_AXIS_X_SRC          1           /* 机身 X 轴源自芯片通道 1 (OUTY) */
#define MAG_AXIS_X_SIGN         1           /* 极性：正向 */
#define MAG_AXIS_Y_SRC          0           /* 机身 Y 轴源自芯片通道 0 (OUTX) */
#define MAG_AXIS_Y_SIGN        -1           /* 极性：反向 */
#define MAG_AXIS_Z_SRC          2           /* 机身 Z 轴源自芯片通道 2 (OUTZ) */
#define MAG_AXIS_Z_SIGN        -1           /* 极性：反向 */

/* ==================== 传感器失效容错 ==================== */
#define SENSOR_FAIL_LIMIT       3           /* 连续失败 N 次才标记通道不可用；偶发失败保留旧数据 */

/* ==================== 气压计（BMP388） ==================== */
#define BARO_I2C_ADDR_A         0x76
#define BARO_I2C_ADDR_B         0x77
#define BARO_CHIP_ID            0x50
#define BARO_OSR_VAL            0x03        /* OSR(0x1C): osr_p=011(×8), osr_t=000(×1) */
#define BARO_ODR_VAL            0x03        /* ODR(0x1D): odr_sel=3 → 25Hz */
#define BARO_IIR_VAL            0x04        /* CONFIG(0x1F): iir_filter=010 → 系数 3 */
#define BARO_PWR_VAL            0x33        /* PWR_CTRL(0x1B): normal 模式 + press_en + temp_en */

/* ==================== 任务 ==================== */
#define TASK_STACK_SENSOR       4096
#define TASK_STACK_DIAGNOSTIC   4096
#define TASK_STACK_GNSS         4096
#define TASK_STACK_INPUT        4096
#define TASK_STACK_UI           8192
#define TASK_PRIO_SENSOR        8
#define TASK_PRIO_DIAGNOSTIC    4
#define TASK_PRIO_GNSS          6
#define TASK_PRIO_INPUT         4
#define SENSOR_LOOP_MS          50          /* 采样周期 20 Hz */
#define DIAG_BOOT_PERIOD_MS     1000        /* 启动自检 5 次，每秒 1 次 */
#define DIAG_BOOT_COUNT         5
#define DIAG_HEARTBEAT_MS       5000

/* ==================== I2C 超时 ==================== */
#define I2C_XFER_TIMEOUT_MS     100
#define I2C_PROBE_TIMEOUT_MS    50
