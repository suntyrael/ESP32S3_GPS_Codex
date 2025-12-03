#ifndef COMPONENT_PGEAR_H
#define COMPONENT_PGEAR_H

/**
 * @file pgear.h
 * @brief P-GEAR 计时与页面逻辑接口。
 */

#include "esp_err.h"

/**
 * @brief 初始化 P-GEAR 计时器。
 */
esp_err_t pgear_init(void);

/**
 * @brief 更新计时逻辑。
 */
esp_err_t pgear_tick(void);

#endif // COMPONENT_PGEAR_H
