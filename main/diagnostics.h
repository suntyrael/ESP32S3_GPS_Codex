/*
 * diagnostics.h - 诊断与自检（阶段 1：传感器自检；GNSS/UI 后续扩展）
 * 格式：[DIAG][T=xxxxms] ... RESULT: OK/FAIL
 */
#pragma once

/** @brief 启动自检任务：前 5 秒每秒自检一次，之后每 5 秒心跳 */
void diagnostics_task(void *arg);

/** @brief 输出一次完整自检（供任务与事件触发复用） */
void diagnostics_report_boot(void);
