#ifndef COMPONENT_LOG_H
#define COMPONENT_LOG_H

/**
 * @file log.h
 * @brief 心跳与事件日志接口。
 */

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief 输出心跳日志。
 */
esp_err_t log_heartbeat(void);

/**
 * @brief 记录一条事件日志。
 */
esp_err_t log_event(const char *message, uint32_t timestamp_ms);

#endif // COMPONENT_LOG_H
