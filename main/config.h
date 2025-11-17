#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 按键时序定义（毫秒）
#define CONFIG_BUTTON_DEBOUNCE_MS      50
#define CONFIG_BUTTON_SHORT_MS         50
#define CONFIG_BUTTON_MEDIUM_MS        500
#define CONFIG_BUTTON_LONG_MS          2000

// 旋转编码器滤波
#define CONFIG_ENCODER_STEP_WINDOW     3
#define CONFIG_ENCODER_IDLE_CLEAR_MS   500

// P-Box 配置
#define CONFIG_PBOX_START_SPEED_KMH    1.0f
#define CONFIG_PBOX_START_ACCEL_G      0.15f
#define CONFIG_PBOX_TARGET_SPEED_KMH   100.0f

// GNSS 设置
#define CONFIG_GNSS_DEFAULT_RATE_HZ    10
#define CONFIG_GNSS_MIN_SATS_LOCK      4

// 记录功能
#define CONFIG_GPX_DIRECTORY           "/GPX"
#define CONFIG_GPX_FILE_PREFIX         "ACT_"

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
