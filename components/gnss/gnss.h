#ifndef COMPONENT_GNSS_H
#define COMPONENT_GNSS_H

/**
 * @file gnss.h
 * @brief GNSS 模块接口，负责 UART1 初始化、波特率配置以及 NMEA/UBX 数据解析。
 */

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GNSS 配置结构体。
 */
typedef struct {
    int uart_port;           /**< UART 端口号，默认使用 UART1。 */
    int tx_pin;              /**< GNSS 模块 TXD 引脚。 */
    int rx_pin;              /**< GNSS 模块 RXD 引脚。 */
    int baudrate;            /**< GNSS 模块波特率。 */
    int pps_pin;             /**< PPS 信号输入引脚。 */
    bool enable_ubx;         /**< 是否启用 UBX 二进制解析。 */
} gnss_config_t;

/**
 * @brief GNSS 解析得到的位置数据。
 */
typedef struct {
    double latitude;         /**< 纬度，单位度。 */
    double longitude;        /**< 经度，单位度。 */
    double altitude;         /**< 高度，单位米。 */
    float speed_kmh;         /**< 速度，单位 km/h。 */
    float heading_deg;       /**< 航向角，单位度。 */
    uint64_t timestamp_ms;   /**< 时间戳，毫秒。 */
    bool fix_valid;          /**< 是否锁定卫星。 */
} gnss_fix_t;

/**
 * @brief 初始化 GNSS 驱动。
 * @param config 硬件配置。
 * @return 执行结果。
 */
esp_err_t gnss_init(const gnss_config_t *config);

/**
 * @brief 轮询 GNSS 数据并解析。
 * @param out_fix 输出解析结果。
 * @return 执行结果。
 */
esp_err_t gnss_poll_data(gnss_fix_t *out_fix);

/**
 * @brief 配置 GNSS 波特率和输出模式。
 * @param baudrate 新波特率。
 * @return 执行结果。
 */
esp_err_t gnss_configure_baudrate(int baudrate);

#ifdef __cplusplus
}
#endif

#endif // COMPONENT_GNSS_H
