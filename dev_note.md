# 开发说明（dev_note）

本文档是 `ESP32S3_GPS_Codex` 固件的**开发需求与实现规格**：任务拓扑、数据契约、状态机、时序与并发约束，以及易踩坑清单。与 README.md（产品/硬件规格）配套使用。**编码前先通读本文档 §11 与 §12。**

---

## 1. 系统任务与责任

| 任务 | 栈/优先级 | 责任 | 关键接口 |
| --- | --- | --- | --- |
| `sensor_task` | 4096 / 8 | 驱动 `sensors_update()` 采样 IMU/GNSS/气压/电源，累积骑行/轨迹/P-Box 指标，推送 GPX 样本 | `sensors_update`, `sensors_get_state`, `gpx_logger_push_sample` |
| `diagnostic_task` | 4096 / 4 | 启动 5 s 每秒一次详细自检，之后 5 s 心跳 | `diagnostics_report_boot`, `diagnostics_report_heartbeat` |
| `input_task` | 4096 / 9 | 消费 `input_manager` 事件，负责模式切换、轨迹开关、P-Box 状态 | `input_manager_get_event`, `gpx_logger_start/stop` |
| `ui_task` | 8192 / 6 | 驱动 LVGL 视图层，刷新状态栏与五大模式界面 | `ui_*` 模块、`lv_timer_handler` |
| `input_manager` 内部任务 | 4096 / 9 | GPIO 中断读编码器 + 轮询按键：3-step 滤波、500 ms 空闲清零、短/中/长/双击判定 | `diagnostics_trigger_event` |
| `gpx_task` | 4096 / 5 | 处理 GPX 记录队列，按需写 SD | `gpx_logger_*` |

> **约束 D-01（优先级纪律）**：优先级从高到低 `input(9) > sensor(8) > ui(6) > gpx(5) > diagnostic(4)`，禁止调整；高优先级任务**禁止**持锁等待低优先级任务（防优先级反转）。

---

## 2. 数据结构与共享状态（数据契约）

- `sensors_state_t`：IMU、磁力计、气压计、电源、GNSS（含卫星数组 ≤32 颗）的统一快照。
  - **契约**：`sensors_get_state()` 返回**按值拷贝**；调用方禁止保存返回指针跨任务使用；`sensors.c` 内部用互斥锁保护。
- `system_context_t`：应用层运行态——模式枚举 `ui_mode_t`、骑行/轨迹里程与时间累计、P-Box 状态机（目标速度/已用时间/启动 tick）、设置菜单选项与 GNSS 刷新率。
- `ui_telemetry_t`：UI 刷新入参，封装 `sensors_state_t` 与 GPX 录制状态。

> **约束 D-02（所有权）**：每个结构体只允许一个写者（`sensor_task` 写 `sensors_state_t`，`input_task` 写模式/P-Box 状态），其余任务只读；跨任务传递用 FreeRTOS 队列（GPX 样本队列、输入事件队列、UI 更新队列）。

> **约束 D-03（NVS 命名空间）**：
> - `settings_store`：GNSS 刷新率、星座掩码、动态模式、P-Box 阈值、亮度。
> - `cal`：IMU 零偏、磁力计硬铁/软铁系数。
> - key 命名：小写下划线、≤15 字符；**只在值变化时写入**；GNSS 命令 ACK 成功后再持久化（先下发后落盘）。

---

## 3. 模式与输入映射

- 旋转编码器（左/右）：
  - 非设置模式：`MODE_BIKE → MODE_GPS_LOGGER → MODE_PBOX → MODE_GNSS_INFO → MODE_SETTINGS` 循环。
  - 设置模式：上下移动高亮项 `settings_option_t`。
- 按键：
  - 短按：P-Box 模式下 READY ↔ ARMED/FINISHED 切换。
  - 中按（~500 ms）：切换 GPX 记录 start/stop。
  - 长按（~2000 ms）：设置界面 ↔ 主界面切换。
  - 双击（间隔 ≤400 ms）：预留应用层扩展。

> **约束 D-04（输入时序）**：消抖 50 ms、3-step 滤波、500 ms 无变化清零、中按 500 ms、长按 2000 ms——**全部以 `config.h` 宏为准**，禁止散落魔数；事件经队列投递给 `input_task`；ISR 只置标志，不做业务。

---

## 4. P-Box 状态机

| 状态 | 进入条件 | 退出条件 | 行为 |
| --- | --- | --- | --- |
| READY | 默认 / FINISHED 后短按 | 短按 → ARMED | 等待用户指令 |
| ARMED | READY + 短按 | 满足启动条件 → RUNNING | 持续监测启动条件 |
| RUNNING | ARMED + 启动条件满足 | GPS 速度 ≥ 目标速度（默认 100 km/h）→ FINISHED | 累积计时 `pbox_elapsed_s` |
| FINISHED | RUNNING + 达标 | 短按 → READY | UI 显示 “TEST FINISHED!!!” |

- **启动条件**：GPS 速度 < 1 km/h **且** IMU X 轴线性加速度 > 阈值（默认 0.15 G，可调 0.10~0.30 G）。
- **计时精度**：用 `esp_timer_get_time()`（μs），禁止 tick 换算。
- **IMU 轴向**：LSM6DSR 为 Z 反向、Y 不变；若安装方向不同，换轴逻辑必须参数化（`config.h`），禁止硬编码轴号。
- 启动判定阈值实时读取设置值（`s_ctx.pbox_start_accel_g`），状态机判断时禁止缓存过期值。

---

## 5. UI 结构

- 顶部常驻 `ui_state_bar`：卫星数、电量、充电状态。
- 5 个主界面均为 `lv_scr_act()` 子节点，`refresh_ui()` 按当前模式隐藏/显示。
- 每界面独立 `ui_*.c` 文件（`ui_state_bar / ui_bike_computer / ui_gps_logger / ui_pbox / ui_gnss_info / ui_settings`）。
- 字体、尺寸、布局常量集中 `ui_common.h`。

> **约束 D-05（LVGL 线程模型）**：全部 LVGL API 只允许在 `ui_task` 内调用；其他任务通过队列/标志请求 UI 更新，禁止直接调 `lv_*`；`lv_timer_handler` 轮询周期 5~10 ms。
> **约束 D-06（刷新频率）**：状态栏 ~1 Hz；速度/仪表 5~10 Hz；卫星列表随 GNSS 刷新率。禁止在每帧做重计算（如路径字符串拼接）。

---

## 6. 诊断与日志

- `diagnostics_init()` 初始化 UART0 日志通道。
- 启动 5 s：`diagnostics_report_boot()` 每秒输出 GNSS DOP、卫星 CN0、IMU/气压/磁力计/电源温度等全量信息。
- 之后每 5 s：`diagnostics_report_heartbeat()` 记录卫星数、速度、电量、温度。
- 输入事件：`input_manager` 生成事件时调用 `diagnostics_trigger_event()`，同步打印输入节奏与按压时长。
- 统一格式：`[DIAG][T=xxxxms] ... RESULT: OK/FAIL`。

> **约束 D-07（日志非阻塞）**：日志走 ESP_LOG 通道，禁止在日志路径做文件 I/O 或持锁；诊断只读共享快照。

---

## 7. 记录与存储（GPX）

- `gpx_logger` 事件队列驱动独立任务：
  - **START**：创建 `/GPX/ACT_xxxx.gpx`，写 `<gpx><metadata><time>`。
  - **SAMPLE**：写 `<trkpt>`（坐标/海拔/`<time>`）+ `<extensions>`（温度、三轴/总 G、气压、电池百分比、电压、模式/上下文含 P-Box 状态、骑行/轨迹距离、P-Box 计时）。
  - **STOP**：闭合标签并关闭文件。
- 队列深度由 `CONFIG_GPX_SAMPLE_QUEUE_DEPTH` 控制；采集任务仅在 `GPX_LOGGER_STATE_RECORDING` 时推送快照；**每个样本 `fflush`** 降低断电丢数据风险。
- 距离与时间累计逻辑与文件写入保持一致（同一份累计状态，禁止两处计算）。

> **约束 D-08（写盘纪律）**：`gpx_task` 是唯一写 SD 的任务；SD 挂载失败/写失败只置状态栏图标并记日志，**禁止阻塞 `sensor_task` 或 UI**；STOP 后必须 `fclose`，卸载前 `fflush`。
> **约束 D-09（时间格式）**：GPX `<time>` 一律 `YYYY-MM-DDTHH:MM:SSZ`（UTC）；无有效 GNSS 时间时禁止写伪造时间戳。

---

## 8. GNSS 数据路径

- `gnss_init()`：LDO 使能（GPIO14 高）→ 延时 ≥100 ms → **速率探测**：按 [9600 → 38400 → 115200] 依次试探，收到有效 NMEA 或 ACK 即锁定（板载 ATGM336H-F8N76 默认 9600；若贴 NEO-M9N-00B 则默认 38400）→ `PMTK251,115200` 切换。
- 互斥保护的 `send_command_with_ack()` / `send_ubx_with_ack()` 并行下发 PMTK 与 UBX：`PMTK251/220/353`、`UBX-CFG-RATE/GNSS/NAV5`。
- 启动后按 `settings_store` 应用刷新率 / 星座掩码 / 动态模式：`gnss_set_update_rate()`、`gnss_set_constellations()`、`gnss_set_dynamic_mode()`。
- `gnss_poll()`：获取互斥后非阻塞读 UART，解析 `GGA/RMC/GSA/GSV`，更新 DOP、卫星列表、在用卫星数；RMC 时间戳转 `time_t`（2 位年 → 80 年窗口）。
- UBX ACK/NAK 按 (class, msgID) 匹配并写诊断日志。

> **约束 D-10（解析正确性）**：
> - NMEA 行缓冲 ≥128 B；UART RX ring buffer ≥ 2× 最大句长；**禁止在中断里做字符串解析**。
> - **GSV 多句累积**：按 total/sentence index + talker 分别累积，卫星数组上限 32，跨 talker 清空重开。
> - 校验和失败 / RMC 无效（`A` 标志缺失）/ 无 fix 的时间一律丢弃。
> - 配置命令独立等待 ACK，**超时 ≤500 ms**，失败重试 1 次后降级（保持上次可用配置），禁止阻塞式死等。
> - 双协议只采纳有 ACK 的一方；PMTK 用 `$PMTK001` 应答确认。

---

## 9. 传感器与校准

- `sensors_init()`：初始化 I2C/ADC 与充电状态 GPIO；依次唤醒 LSM6DSR、LIS2MDL、BMP388；校验芯片 ID（见 README C-02）；校准数据从 NVS `cal` 读取，缺省用零偏 + 单位软铁系数。
- `sensors_update()`：IMU 应用轴向翻转 + 偏移；磁力计 X/Y 交换、Y/Z 反向并按硬铁/软铁系数缩放；BMP388 官方补偿公式；电源用 oneshot ADC + line fitting（饱和保护见 README §3.4）。
- `sensors_start_calibration()` 后台 FreeRTOS 任务：IMU 采 512 个静止样本求均值；磁力计采 600 个 8 字摇晃样本求偏置 + 软铁系数；进度/提示通过 `sensors_calibration_status_t` 暴露给 UI；完成后写 NVS `cal`。

> **约束 D-11（校准状态机）**：校准期间禁止切换模式/停止采集；进度必须单调；UI 提示 “保持静止” / “8 字晃动” 与百分比实时同步；校准完成写 NVS 成功后才允许退出。

---

## 10. 设置菜单与持久化

- `settings_store`：NVS 保存 GNSS 刷新率、星座组合、动态模式、P-Box 启动阈值；`app_main` 启动时同步到 `system_context_t` 并立即应用。
- `apply_settings_action()` 短按执行：
  - **GNSS 刷新率**：1/5/10/25 Hz 循环，`PMTK220` + `UBX-CFG-RATE`，ACK 成功才写 NVS。
  - **动态模式**：步行/汽车/海上/航空循环，`UBX-CFG-NAV5`，结果写诊断事件。
  - **星座组合**：GPS+GLONASS/Galileo/BeiDou 组合循环，`PMTK353` + `UBX-CFG-GNSS`，日志输出 ACK/NAK。
  - **IMU/磁力计校准**：`sensors_start_calibration()`，UI 读状态与进度。
  - **P-Box 阈值**：0.10~0.30 G 循环，实时影响状态机并持久化。
- `diagnostics_trigger_event()` 在每次配置下发后打印结果。

---

## 11. 开发约束清单（编码硬性要求）

| 编号 | 约束 |
| --- | --- |
| **D-01** | 任务优先级 `input(9) > sensor(8) > ui(6) > gpx(5) > diagnostic(4)`，禁止调整；防优先级反转 |
| **D-02** | 结构体单一写者；跨任务传递只走队列/快照拷贝 |
| **D-03** | NVS 命名空间与 key 规范；值变化才写；ACK 成功后才持久化 |
| **D-04** | 输入时序全走 `config.h` 宏；ISR 只置标志 |
| **D-05** | LVGL 单线程（`ui_task`）；跨任务 UI 更新走队列/标志 |
| **D-06** | 刷新频率分级（状态栏 1 Hz / 仪表 5-10 Hz）；禁止每帧重计算 |
| **D-07** | 日志非阻塞，禁止日志路径持锁或做文件 I/O |
| **D-08** | `gpx_task` 唯一写 SD；SD 失败只降级不阻塞；STOP 必 `fclose` |
| **D-09** | GPX 时间一律 UTC ISO8601，禁止伪造时间戳 |
| **D-10** | GNSS 解析：多句 GSV 累积、校验和、无效时间丢弃、ACK 超时 ≤500 ms、失败降级不阻塞 |
| **D-11** | 校准状态机：期间锁定模式，进度单调，NVS 写成功才退出 |
| **D-12** | **零魔数**：所有可调参数 `config.h`，UI 常量 `ui_common.h`，字符串 `strings.h` |
| **D-13** | 编译开 `-Wall -Werror`；头文件 `#pragma once`；函数前缀按模块 |
| **D-14** | 任何 `esp_err_t` 返回值必须处理；禁止 `(void)ret` 静默丢弃 |
| **D-15** | 大缓冲进 PSRAM（LVGL draw buffer / LCD DMA 描述符除外——按 ESP32-S3 约束），任务栈 ≤8 KB 进内部 RAM |
| **D-16** | 关键不变量用断言（边界、数组下标、状态机合法迁移），生产可关 |

---

## 12. 易踩坑清单（编码前逐条核对）

| # | 坑 | 对策 |
| --- | --- | --- |
| 1 | **GPIO26~32 被封装内 Flash/PSRAM 占用**（四线模式） | 引脚分配时先查 README §3.3 红线表 |
| 1b | **误配 Octal PSRAM 会吞掉 GPIO33~37**（与 SD 冲突） | `CONFIG_SPIRAM_MODE_QUAD=y`，禁止 OCT（README §3.3-2 / §8.2） |
| 2 | **SD 引脚表曾自相矛盾（旧文档）** | **已按原理图核实**：CLK=36/CMD=35/D0=37/D1=38/D2=34/D3=33（README §3.2），编码以 `config.h` 宏为准 |
| 3 | **GPIO3 是 strapping 引脚**（JTAG 源选择，规格书表 3-1：复位默认**浮空**） | 靠外部上拉保证复位为高；禁止改变时序/下拉 |
| 3b | **GPIO10=CHIP_PU 网络 / GPIO11=WATCHDOG（Q3/Q4）** | 用途待确认；禁止主动驱动 CHIP_PU；看门狗翻转逻辑确认后再写 |
| 4 | **GPIO12=ADC2_CH1 且 1:1 分压 4.2 V 超量程** | 核实分压比；固件饱和保护；将来开 Wi-Fi 必须迁 ADC1 |
| 5 | **ATGM336H（PMTK/9600）与 NEO-M9N（UBX/38400）双协议双波特率** | 双协议并行下发只认 ACK 方；波特率探测 [9600→38400→115200]（README §3.5） |
| 5b | **IMU 实为 LSM6DSVETR（非 LSM6DSR），气压计实为 BMP390L（非 BMP388）** | 芯片 ID 宽松校验：LSM6DSR=0x6B（勿写 0x6A）、LIS2MDL=0x40、BMP=0x50/0x60 都接受；待上传 LSM6DSV 规格书 |
| 6 | **GSV 多句漏累积** | 按 total/index + talker 累积，上限 32，跨 talker 重置 |
| 7 | **NMEA 年份 2 位 → 时间戳错误** | 80 年滚动窗口（2000~2079），无效 fix 时间丢弃 |
| 8 | **LVGL 跨任务调用崩溃** | 全部 LVGL 调用锁在 `ui_task` |
| 9 | **等待 ACK 无超时 → 死等** | 所有等待 ≤500 ms 超时 + 重试 1 次 + 降级 |
| 10 | **高优先级持锁低速 I/O → 优先级反转** | 按 D-01，锁内禁止低速 I/O |
| 11 | **两处累计距离/时间 → 数据不一致** | 单份累计状态（§7），写文件与显示同一来源 |
| 12 | **ADC 未校准读值非线性** | oneshot + line fitting 校准；禁止直接读原始值 |
| 13 | **I²C 1 MHz 无外部上拉不稳** | 确认上拉；调试期降 400 kHz |
| 14 | **NVS 频繁写磨损** | 值变化才写；配置 ACK 成功才落盘 |
| 15 | **升级 LVGL v9 / 漂移 IDF 版本** | 锁定 8.3.11 / v6.1.0，`dependencies.lock` 入库 |

---

## 13. 后续扩展建议

1. **轨迹可视化**：GPS 记录界面叠加折线轨迹或缩略地图，提升现场复盘能力（注意 UI 刷新频率约束 D-06）。
2. **传感器融合**：Kalman 融合 IMU+GNSS 推算姿态/坡度，提升 P-Box 与骑行分析精度。
3. **连接能力**：BLE/Wi-Fi/USB CDC 传输 GPX 或实时数据 + OTA；注意 GPIO19/20 已预留、Wi-Fi 开启后 ADC2 不可用（README §3.4）。
