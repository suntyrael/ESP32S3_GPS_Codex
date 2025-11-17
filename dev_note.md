# 开发说明（dev_note）

本说明文档概述 `ESP32S3_GPS_Codex` 固件的核心逻辑、任务拓扑以及各模块间的数据流，便于后续扩展与调试。

## 1. 系统任务与责任

| 任务 | 堆栈/优先级 | 责任 | 关键接口 |
| --- | --- | --- | --- |
| `sensor_task` | 4096 / 8 | 驱动 `sensors_update()` 采样 IMU/GNSS/气压/电源信息，并累积骑行/轨迹/P-Box 指标，推送 GPX 样本。 | `sensors_update`, `sensors_get_state`, `gpx_logger_push_sample` |
| `diagnostic_task` | 4096 / 4 | 启动阶段（5s）每秒输出一次详细自检，之后进入 5s 心跳。 | `diagnostics_report_boot`, `diagnostics_report_heartbeat` |
| `input_task` | 4096 / 9 | 读取 `input_manager` 事件，负责模式切换、轨迹开关、P-Box 状态。 | `input_manager_get_event`, `gpx_logger_start/stop` |
| `ui_task` | 8192 / 6 | 驱动 LVGL 视图层，刷新状态栏与五大模式界面。 | `ui_*` 模块、`lv_timer_handler` |
| `input_manager` 内部任务 | 4096 / 9 | 使用 GPIO 中断读取旋转编码器 + 轮询按键，完成 3-step 滤波、500ms 空闲清零与短/中/长/双击判定。 | `diagnostics_trigger_event` |
| `gpx_task` | 4096 / 5 | 处理 GPX 记录队列，按需写入 SD（当前示例以日志代替）。 | `gpx_logger_*` |

## 2. 数据结构与共享状态

- `sensors_state_t`：集中描述 IMU、磁力计、气压计、电源、GNSS（含卫星数组）数据，通过 `sensors_get_state()` 复制到调用者。
- `system_context_t`：应用层运行态，包含：
  - 模式枚举 `ui_mode_t`
  - 骑行与轨迹里程/时间累计
  - P-Box 状态机（目标速度、已用时间、启动 tick）
  - 设置菜单选项与 GNSS 刷新率
- `ui_telemetry_t`：UI 刷新入参，封装 `sensors_state_t` 与 GPX 录制状态。

所有任务通过调用 `sensors_get_state` 获取一致快照，避免直接访问硬件缓冲。`sensors.c` 内部通过互斥锁保护状态，并在 `sensors_init()` 中唤醒 GNSS UART/NMEA 管线。需要持久化的配置项统一收敛在 `config.h`，以便菜单与固件同步更新。

## 3. 模式与输入映射

- 旋转编码器（左/右）：
  - 非设置模式：`MODE_BIKE → MODE_GPS_LOGGER → MODE_PBOX → MODE_GNSS_INFO → MODE_SETTINGS` 循环。
  - 设置模式：上下移动高亮项 `settings_option_t`。
- 按键：
  - 短按：P-Box 模式下在 READY ↔ ARMED/FINISHED 间切换。
  - 中按：切换 GPX 记录（start/stop）。
  - 长按：在设置界面与主界面之间切换。
- 双击：按键双击（<=400ms 间隔）留给应用层扩展。

## 4. P-Box 状态机

| 状态 | 进入条件 | 退出条件 | 行为 |
| --- | --- | --- | --- |
| READY | 默认状态或 FINISHED 后短按 | 短按进入 ARMED | 等待用户指令 |
| ARMED | READY + 短按 | 满足启动条件（GPS 速度 <1 km/h 且 IMU X 加速度 >0.15 G）进入 RUNNING | 持续监测启动条件 |
| RUNNING | ARMED + 满足启动条件 | GPS 速度 ≥ 目标速度（默认 100 km/h） | 累积计时 `pbox_elapsed_s` |
| FINISHED | RUNNING + 达标 | 短按回到 READY | 在 UI 中显示 TEST FINISHED |

## 5. UI 结构

- 顶部常驻 `ui_state_bar`，显示卫星数、电量、充电状态。
- 5 个主界面均在 `lv_scr_act()` 子节点内创建，`refresh_ui()` 根据当前模式隐藏/显示。
- 每个界面拥有独立的 `ui_*.c` 源文件，符合 README 对可维护性要求。
- LVGL 字体、尺寸、布局常量集中在 `ui_common.h`。

## 6. 诊断与日志

- `diagnostics_init()` 初始化日志通道。
- 启动阶段（5s）调用 `diagnostics_report_boot()` 输出 GNSS DOP、卫星 CN0、IMU/气压/磁力计/电源温度等全量信息，格式贴近 SRS 示例。
- 之后每 5s 调用 `diagnostics_report_heartbeat()`，记录卫星数、速度、电量、温度。
- `input_manager` 在生成事件时调用 `diagnostics_trigger_event()`，串口可同步查看输入节奏与按压时长。

## 7. 记录与存储

- `gpx_logger` 使用队列缓存传感器快照，示例中以日志代替实际 SD 写入，后续可在任务中调用 FATFS / VFS API 落盘。
- 轨迹统计（距离、时间）仅在 `GPX_LOGGER_STATE_RECORDING` 时累积，保证与文件一致。

## 8. GNSS 数据路径

- `gnss_init()` 通过 GPIO 使能 LDO，配置 UART1 并发送 `PMTK251` 切换到 115200bps，随后根据 `CONFIG_GNSS_DEFAULT_RATE_HZ` 生成 `PMTK220` 指令。每条指令都会等待 `PMTK001` ACK 并输出日志，满足 F-SYS-04。
- `gnss_poll()` 非阻塞读取 UART，将字节流切分为 NMEA 语句并解析 `GGA/RMC/GSA/GSV`：
  - `GGA` 提供经纬度、海拔、HDOP、在用卫星数。
  - `RMC` 追加速度、定位状态。
  - `GSA` 更新 PDOP/VDOP，并记录当前使用的卫星编号。
  - `GSV` 按星座更新每颗卫星的 CN0/仰角/方位以及状态颜色。
- 解析结果落在 `sensors_state_t.gnss`，供 UI、P-Box、诊断日志和轨迹记录统一消费。

## 9. 后续扩展建议

1. **硬件驱动接入**：在 `sensors_update()` 中将 IMU/磁力计/气压/电源示例数据替换为实际 I2C/SPI 读数，并接入 NVS 校准。
2. **GNSS 配置菜单**：在设置界面根据用户选择生成 PMTK/UBX 指令，重用 `gnss_poll()` 的 ACK 逻辑并写入 `diagnostics_trigger_event`。
3. **GPX 写入**：扩展 `gpx_task`，以 `/GPX/ACT_*.gpx` 命名规则创建文件，写入 `<extensions>` 字段记录温度、G 值等附加信息。
4. **NVS 配置持久化**：将 GNSS 刷新率、P-Box 阈值等可调参数保存至 NVS，启动时加载并同步至 UI。

