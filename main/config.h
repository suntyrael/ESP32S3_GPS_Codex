#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

// 按键时序定义（毫秒）
#define CONFIG_BUTTON_DEBOUNCE_MS      50
#define CONFIG_BUTTON_SHORT_MS         50
#define CONFIG_BUTTON_MEDIUM_MS        500
#define CONFIG_BUTTON_LONG_MS          2000
#define CONFIG_BUTTON_DOUBLE_GAP_MS    400

// 输入引脚
#define CONFIG_BUTTON_GPIO             2
#define CONFIG_ENCODER_GPIO_A          1
#define CONFIG_ENCODER_GPIO_B          3

// GNSS UART 引脚
#define CONFIG_GNSS_UART_TX            17
#define CONFIG_GNSS_UART_RX            18
#define CONFIG_GNSS_LDO_EN             14

// I2C 总线
#define CONFIG_SENSOR_I2C_PORT         0
#define CONFIG_SENSOR_I2C_SCL          39
#define CONFIG_SENSOR_I2C_SDA          40
#define CONFIG_SENSOR_I2C_FREQ_HZ      1000000

// 旋转编码器滤波
#define CONFIG_ENCODER_STEP_WINDOW     3
#define CONFIG_ENCODER_IDLE_CLEAR_MS   500

// P-Box 配置
#define CONFIG_PBOX_START_SPEED_KMH    1.0f
#define CONFIG_PBOX_START_ACCEL_G      0.15f
#define CONFIG_PBOX_TARGET_SPEED_KMH   100.0f
#define CONFIG_PBOX_ACCEL_STEP_G       0.05f

// GNSS 设置
#define CONFIG_GNSS_UART_PORT          1
#define CONFIG_GNSS_UART_BUFFER        4096
#define CONFIG_GNSS_DEFAULT_RATE_HZ    10
#define CONFIG_GNSS_MIN_SATS_LOCK      4
#define CONFIG_GNSS_DEFAULT_BAUD       115200

// 电源检测
#define CONFIG_BATTERY_ADC_CHANNEL     ADC_CHANNEL_1
#define CONFIG_BATTERY_CHARGE_GPIO     21

// 记录功能
#define CONFIG_GPX_DIRECTORY           "/GPX"
#define CONFIG_GPX_FILE_PREFIX         "ACT_"
#define CONFIG_GPX_NAMESPACE           "esp"
#define CONFIG_GPX_SAMPLE_QUEUE_DEPTH  32

// 诊断日志
#define CONFIG_DIAG_BOOT_INTERVAL_MS   1000
#define CONFIG_DIAG_HEARTBEAT_MS       5000
#define CONFIG_DIAG_BOOT_DURATION_MS   5000

// 任务堆栈/优先级
#define CONFIG_TASK_STACK_DEFAULT      4096
#define CONFIG_TASK_PRIO_SENSOR        8
#define CONFIG_TASK_PRIO_INPUT         9
#define CONFIG_TASK_PRIO_UI            6
#define CONFIG_TASK_PRIO_LOGGER        5
#define CONFIG_TASK_PRIO_DIAG          4

#ifdef __cplusplus
}
#endif
