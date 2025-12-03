#ifndef COMPONENT_GPX_H
#define COMPONENT_GPX_H

/**
 * @file gpx.h
 * @brief GPX 文件写入接口，预留轨迹输出实现。
 */

#include "esp_err.h"
#include "gnss.h"

/**
 * @brief 初始化 GPX 文件。
 */
esp_err_t gpx_begin(void);

/**
 * @brief 写入 GNSS 轨迹点。
 */
esp_err_t gpx_append_fix(const gnss_fix_t *fix);

/**
 * @brief 结束并关闭 GPX 文件。
 */
esp_err_t gpx_close(void);

#endif // COMPONENT_GPX_H
