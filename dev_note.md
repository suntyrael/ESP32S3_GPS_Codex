# 开发说明（dev_note）

本文档是 `ESP32S3_GPS_Codex` 固件的**开发需求与实现规格**：任务拓扑、数据契约、状态机、时序与并发约束，以及易踩坑清单。与 README.md（产品/硬件规格）配套使用。**编码前先通读本文档 §11 与 §12。**

> 📚 **文档导航**：
> - `README.md` — 产品/硬件规格、引脚表、编码约束 C-01~C-17
> - `dev_note.md`（本文）— 任务拓扑、数据契约、开发约束 D-01~D-16、踩坑清单
> - `DOC/ARCHITECTURE.md` — **驱动框架**（总线/驱动/汇聚/业务四层 + 版本 CI 策略）
> - `DOC/DEV_LOG/` — **开发日志**（独立保存，随 git 版本化，每次 commit 同步）
> - `DOC/SPEC/` — 器件规格书归档

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

## 3. 模式与输入映射（V0.3.0 3 大页面架构与设置页分层状态机）

- 旋转编码器（左旋 CCW / 右旋 CW）：
  - **默认页面浏览态（PAGE_NAV）**：循环切换 3 个大页面：`[Page 0: 主功能页] ↔ [Page 1: 传感器诊断页] ↔ [Page 2: 系统设置页]`。
  - **设置页光标选择态（CURSOR_NAV）**：在 Page 2 内部上下平滑移动光标选择条目（Item 1~9），视口自动随选平滑滚动（`lv_obj_scroll_to_view`）。
  - **Function 模式编辑态（ITEM_EDIT）**：在 Item 1 下轮转切换 `P-GEAR` ↔ `TRACK REC` ↔ `BIKE COMP`。
- Page 0 的子视图模式由系统设置 `FUNCTION MODE` 决定：
  - `MAIN_PAGE_PBOX` (0): P-Box 直线加速测试（默认）
  - `MAIN_PAGE_LOGGER` (1): TRACK REC 轨迹记录仪
  - `MAIN_PAGE_BIKE` (2): BIKE COMP 自行车码表
- 按键输入在各状态下的精确行为映射：
  - **短按（<800ms）**：
    - **Page 0 (P-Box)**：一键复位（RESET），重新进入就绪待机；
    - **Page 0 (轨迹记录)**：短按开启记录（防误触：开启短按）；
    - **Page 0 (码表)**：短按暂停/继续单次骑行；
    - **Page 2 (PAGE_NAV 页面浏览态)**：短按激活“光标选择态（CURSOR_NAV）”，显示焦点选择框；
    - **Page 2 (CURSOR_NAV 光标选择态)**：
      - 光标处于 Item 1 (`FUNCTION MODE`) 时：短按进入 `ITEM_EDIT` 波轮轮转编辑态；
      - 光标处于 Item 2~8 时：短按直接循环步进修改当前项预设值并即时生效；
      - 光标处于 Item 9 (`SENSOR CALIB`) 时：短按进入 `CALIB_FLOW` 独立传感器校准流程（启动 IMU 5s 倒计时）；
    - **Page 2 (ITEM_EDIT Function 编辑态)**：短按确认选中项并退出编辑态，即刻联动更新 Page 0 并回退到光标态；
    - **Page 2 (CALIB_FLOW 校准流程)**：严格限制只有在完成 8 字校准收敛完毕后界面显示“校准 OK”与保存退出提示，短按才将校准矩阵持久化至 NVS 并安全退出；在未完成 8 字校准前短按无效；
  - **长按（≥800ms，按满即触发无需等松手）**：
    - **即刻响应契约**：按下持续满 800ms 瞬间**立即发射事件并执行底层响应，绝不等松手**；松开按键时静默归位，绝不误触短按；
    - **Page 0 (轨迹记录)**：停止记录并安全关闭 GPX 文件落盘（防误触：长按停止）；
    - **Page 0 (码表)**：单次里程与极速清零（Trip Reset）；
    - **Page 1 / Page 2 (任意非校准态)**：随时全局快速返回 Page 0 (HOME)；
    - **Page 2 (CALIB_FLOW 校准流程)**：若未完成校准或需中途放弃，长按随时安全退出取消，回退至设置列表上级；
  - **双击（≤400ms）**：
    - **Page 1 (传感器诊断页)**：健康页双击进入 GNSS 调试页（默认 NMEA LOG）；调试页内双击返回健康页；
    - **Page 1 调试页滚轮**：`NMEA LOG ↔ SAT INFO` 两个子页切换；
    - **Page 1 调试页长按**：仅返回健康页（不返回主页面）；健康页长按仍返回主页面；调试页短按无副作用；
    - 其余页面：预留扩展。

### 3.1 设置页三层交互状态机流程图 (Mermaid)

```mermaid
flowchart TD
    Page0[Page 0: 主功能页] -->|波轮右旋 CW| Page1[Page 1: 传感器诊断页]
    Page1 -->|波轮右旋 CW| Page2_NAV[Page 2: 系统设置页 - 页面浏览态]
    Page2_NAV -->|波轮右旋 CW| Page0

    subgraph Settings_SM [Page 2 系统设置页三层交互状态机]
        direction TB
        Page2_NAV -->|短按波轮| S_Cursor[2. 光标选择态 CURSOR_NAV<br>• 波轮旋转: 1~9 项平滑滚动聚焦<br>• 长按: 返回 Page 0 HOME]

        S_Cursor -->|光标在 Item 1 时短按| S_EditFunc[3A. Function 编辑态 ITEM_EDIT<br>• 波轮旋转: 轮转切换子功能<br>• 短按: 确认选择并退出<br>• 长按: 返回 Page 0 HOME]
        S_EditFunc -->|短按确认| S_Cursor

        S_Cursor -->|光标在 Item 2~8 时短按| S_Cursor

        S_Cursor -->|光标在 Item 9 时短按| S_Calib1[IMU 倒计时 5s 准备期<br>• 提示水平静止放置设备]
        S_Calib1 -->|5s 倒计时结束| S_Calib2[IMU 静止采样校准<br>• 采集陀螺与加速度零偏]
        S_Calib2 -->|IMU 采样完成| S_Calib3[地磁倒计时 5s 准备期<br>• 提示拿起设备准备做8字晃动]
        S_Calib3 -->|5s 倒计时结束| S_Calib4[地磁空中 8 字校准<br>• 未完成严禁提前退出]
        S_Calib4 -->|必须完成8字采样达到100%| S_CalibOK[校准完成<br>• 显示 '校准 OK' 与保存退出提示]
        S_CalibOK -->|短按保存退出| S_Cursor

        S_Calib1 -.->|长按随时取消| S_Cursor
        S_Calib2 -.->|长按随时取消| S_Cursor
        S_Calib3 -.->|长按随时取消| S_Cursor
        S_Calib4 -.->|长按随时取消| S_Cursor
    end

    Page2_NAV -.->|长按 >900ms| Page0
    S_Cursor -.->|长按 >900ms| Page0
    S_EditFunc -.->|长按 >900ms| Page0
```

> **约束 D-04（输入时序与并发隔离）**：消抖 20 ms、短按 <250 ms、长按 >900 ms、双击间隔 <400 ms；**外部输入任务严禁跨线程直接调用 LVGL 控件**，UI 刷新由 `ui_timer_cb`（LVGL 自身任务）周期渲染，彻底杜绝死锁与 Task Watchdog 崩溃。

---

## 4. P-Box 状态机与测速逻辑

| 状态 | 进入条件 | 退出条件 | 行为 |
| --- | --- | --- | --- |
| READY | 默认开机 / 短按复位 | 直接深踩油门起步 → RUNNING | 等待弹射，计时归零 `00.00s` |
| RUNNING | 起步触发（车速≥1.5km/h 或 推力>0.25G） | 车速 ≥ 目标速度（默认 100 km/h）→ FINISHED；<br>或 2.5s 原地未动 → 自动复位 READY | 100Hz 高频采样，0.01s 毫秒精确计时，记录峰值 G |
| FINISHED | 达到 100 km/h 目标速度 | 短按 → READY | 成绩锁定，显示有效坡度与分段用时 |

- **启动条件（直接踩油门即触发）**：
  - 车辆原本处于静止或低速状态（< 3 km/h）；
  - 踩油门产生向前推力（综合动态线性加速度 > 0.25G）或 GPS 车速出现持续抬升（≥ 1.5 km/h），自动触发计时；
  - **防误触假起步保护**：起步后持续 2.5 秒内车速仍 < 2.0 km/h（判定为桌面晃动/手抖误触发），**自动重置回 `READY` 待机，计时清零**。
- **G-Force 雷达仪表**：水平位移 `px` 由横向 X 轴（`g_lat`）驱动，垂直位移 `py` 由纵向 Y 轴（`g_long`）驱动，动态红点居中，峰值 Peak G 实时捕获。
- **计时精度**：微秒级 `esp_timer_get_time()` 驱动，UI 定时器 50Hz (20ms)，呈现真实 **0.01s 毫秒级丝滑步进**。
- **测试数据清零**：初始显示 `--.-- s`，真实跑出 0-60km/h、0-100km/h、1/4 Mile (402m) 时填充并锁定显示。

---

## 5. UI 结构（基于 LVGL 9.5 与 ST7789 240×320 竖屏）

- **顶部常驻状态栏（20px）**：卫星数、定位状态、10Hz 频度、SD 状态（录制红闪烁）、本地时钟（UTC+8 / 系统秒）、电池电压与百分比、充电标志（`CHG`）。
- **底部常驻导航栏（16px）**：3 个页面指示圆点（Page 0/1/2），当前页拉长为高亮小椭圆。
- **中间视口容器（284px）**：
  - **Page 0（主功能页）**：内含 P-Box、轨迹记录仪、码表 3 套完整子视图，由系统设置项动态切换；
  - **Page 1（传感器诊断页）**：LSM6DSR（RAW ACC、**LIN ACC 线性加速度**、GYRO）、LIS2MDL（MAG、**HEADING 电子罗盘 3D 倾角补偿航向与 8 方位**）、BMP388（PRS/T、**三传感器融合平均温度**、ALT）、NEO-M8N（STATUS、POS、**ALT/SPD/HDG 海拔速度与地面航向**、DOP）；四卡片 Y 轴与间隙精细分配，完全不重叠；
  - **Page 2（系统设置页）**：7 大配置条目，采用加大加粗的 **Oswald 14px** 字体，行距优化，末行底部留有 20px 缓冲，绝不贴底截断。
- **全量定制未压缩 Google Fonts 字库**：
  - `font_chakra_petch_48.c`（48px 科技仪表数字）
  - `font_chakra_petch_16.c`（16px 关键数值）
  - `font_oswald_14.c`（14px 卡片标题/设置名称）
  - `font_oswald_12.c`（12px 辅助标签）
  - `font_roboto_mono_12.c`（12px 诊断页等宽对齐数据与时钟）
  - 全字体挂载 `Montserrat_14` 为安全 Fallback，采用 `bitmap_format = 0` 未压缩直读点阵，免解压缩依赖。
  - 分辨率：`lv_display_get_horizontal_resolution()`
  - 定时器：`lv_timer_handler()` 仍在（ui_task 周期调用）
  - 颜色/字体/样式核心 API 不变（`lv_color_hex`、`lv_label_set_text`、`lv_obj_set_style_*`）
  - 底层端口（tick/显示/输入）统一由 `esp_lvgl_port` 封装，禁止手写 lv_display 注册

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

- `gnss_init()`：LDO 使能（GPIO14 高）→ 延时 ≥100 ms → **速率探测**：按 [9600 → 115200 → 38400] 依次试探，收到有效 NMEA/UBX 帧即锁定（NEO-M8N-0-01 与 ATGM336H **上电默认均为 9600**）。
- 锁定 9600 后先执行一次性 **GNSS SELFTEST**（一个固件版本内验证）：
  `UBX@9600 → PUBX,41(0003)@9600 → PUBX,41(0007)@9600 → PUBX,41(0003)@115200 → PUBX,41(0007)@115200 → UBX@115200`，
  输出 `SELFTEST SUMMARY`；任一路径成功才进入完整 UBX 配置。
- 锁定后非阻塞配置状态机（每条命令 ACK/NAK ≤500 ms，失败重试 1 次后降级）：
  `CFG-PRT(115200) → MON-VER → CFG-RATE → CFG-MSG → CFG-GNSS → CFG-NAV5 → CFG-CFG(save)`；
  配置后 poll `CFG-RATE/CFG-GNSS` 回读并打印 `READBACK`。
  - `CFG-PRT` 必须 `mode=0x000008D0`、`baudRate=0x0001C200`，收到 ACK 后才 `uart_set_baudrate(115200)`；
  - `CFG-RATE` 按设置页 `GPS RATE`；`CFG-MSG` 重算 NAV-PVT/NMEA/NAV-SAT 的 rate；
  - `CFG-GNSS` 最多 3 个主星座，NAK 自动降级；`CFG-CFG` 保存 BBR+Flash（掉电后仍每次重新下发）。
- 设置页 `GPS RATE` / `GNSS MODE` 通过 `gnss_set_update_rate()` / `gnss_set_gnss_mode()` 置请求标志，由 `gnss_task` 在配置就绪后下发；ACK 成功才视为生效。
- 解析 `GGA/RMC/GSA/GSV` + `UBX-NAV-PVT`（可选 `NAV-SAT`），更新 DOP、逐星列表、在用卫星数；RMC 时间戳转 `time_t`（2 位年 → 80 年窗口）。
- `NAV-PVT` 使用 `validDate/validTime` 位，time-only fix（fixType=5）也允许对时；`gSpeed/headMot/hMSL` 偏移严格按协议（60/64/36），不再使用 `sAcc/headAcc`。
- 卫星列表：GSV 按 talker 多句累积（GPS/GLONASS/Galileo/BeiDou/QZSS），GSA 标记 `used`；30s 未更新条目自动淘汰；`gnss_get_sat_info()` 提供 ≤32 颗快照。
- UBX ACK/NAK 按 (class, msgID) 匹配并写诊断日志；`gnss_data_t` 暴露 `baud/cfg_state/rate_hz/ttff_ms/frames_per_s/crc_err/module_sw` 供健康页/调试页显示。

### 8.2 星座配置的硬约束（2026-10-23 u-center + UBX 探针实测）

实物身份：**NEO-M8N-0 正品，SPG 2.01 / PROTVER 15.00（2013-10-29），FIS 有外挂 Flash**。
`MON-VER` 能力串：`GPS;SBAS;GLO;BDS;QZSS`（**无 Galileo**）。

用 `tools/ubx_probe.py` 实测得到的四条铁律（均在 COM8 @9600 复现）：

| 实验 | 请求 | 结果 | 结论 |
| --- | --- | --- | --- |
| A | GPS+GLO（2 块） | ACK | 但**回读不变** → 被规范化，等于没生效 |
| B | GPS+GLO+BDS（3 块） | **NAK** | 含 BDS 即被拒 |
| C | GPS+BDS（2 块） | **NAK** | 同上（原固件默认模式，必失败） |
| D | GPS+GLO，`flags=0x00010001` | ACK | flags 的 bit16/bit24 两种写法都接受，**不是** NAK 原因 |
| E | 仅 GPS（1 块） | ACK（首次偶发无应答） | 单星座可写 |
| F | **5 块全列**，只把 SBAS/QZSS 的 enable 位清零 | ACK + **回读生效** | 必须按回读结构全量下发 |

由此得出：

1. **BDS（gnssId=3）不可启用**：banner `GNSS OTP: GPS GLO` 已说明 OTP 只授权 GPS+GLONASS，握手时尽管 MON-VER 列了 BDS 也会 NAK。Galileo 连能力串都没进。
2. **CFG-GNSS 必须“照拄回读结构 + 仅改 enable 位”**：模块回读为 5 块（GPS 8/16、SBAS 1/3、BDS 8/16（默认 disable）、QZSS 0/3、GLO 8/14），只发 2 块会被 ACK 但静默忽略。
3. **`CFG-MSG` 的 ACK 不能当能力判据**：给不存在的 msgId 也回 ACK。`NAV-SAT (0x01 0x35)` 实例 poll **有本体应答**，说明本固件确实支持 NAV-SAT（与“PROTVER 18 才有”的传言不符，以实测为准）。
4. **CFG-PRT 正确写法**：`mode=0x000008C0`（旧代码写的 `0x8D0` 多置了保留位 bit4）、`inProto=0x0007`（UBX+NMEA+RTCM2，旧代码 `0x0003` 丢了 RTCM2）、`outProto=0x0003`。

固件对应修正（`gnss.c`）：
- 新增 `CFG_STEP_GNSSPOLL`：先 poll `CFG-GNSS` 缓存模板（id/res/max/flags 全量），再下发；回读超时则**跳过下发**、保留模块默认（比写错更安全）。
- `CFG_STEP_GNSS` 改为基于模板构造 payload，仅改 `flags.bit0`。
- `cfg_on_failure(CFG_STEP_GNSS)` 回退目标由 `GPS+BDS`（必再 NAK）改为 **`GPS+GLONASS`**。
- 模式枚举重定义：`GPS_GLO_BDS(0，目标)` / `GPS_GLO(1)` / `GPS_ONLY(2)`；UI 标签同步。
- `CFG-PRT` 改为 `mode=0x8C0`、`inProto=0x0007`。

### 8.2.1 UBX 帧校验和 bug（真正的“UBX 无法配置”根因，2026-10-23 定位）

`ubx_send()` 旧实现把 payload 与 header 分开累加：

```c
for (i < len) { ck_a += payload[i]; ck_b += ck_a; }   /* 只算了 payload 的累积 */
ck_a += cls + id + lenLo + lenHi;
ck_b += ck_a;                                          /* CK_B 少算了 4 次 header */
```

后果：**CK_A 恰好正确、CK_B 永远错**，模块对校验错的 UBX 帧一律静默丢弃，于是：
- `SELFTEST UBX@9600=NO`、`UBX probe: poll MON-VER` 无应答（u-center 却能读 → 曾误判为双主机抢线）
- `$PUBX,41` 能改波特率（NMEA 校验正确）→ 出现“模块跳到 115200 但固件读不到 ACK”的怪象
- `CFG-PRT/RATE/MSG/GNSS/NAV5/CFG` 全部永不生效，`cfg_step` 始终停在 FALLBACK

对照验证（Python 复刻两种算法）：

| 帧 | 正确 CK | 旧固件算出 |
| --- | --- | --- |
| MON-VER poll | `0E 34` | `0E 0E` |
| CFG-GNSS poll | `44 D2` | `44 44` |
| CFG-PRT set | `92 B5` | `92 FF` |
| CFG-GNSS set（F/G 变体） | `1D 75` | `1D 28` |

修复：抽出 `ubx_build()`，按 `class → id → len → payload` 顺序累加一次算完；新增启动自检
`ubx_ck_selfcheck()`：MON-VER poll 必须是 `0E 34`，不符就 `ESP_LOGE`（历史回归保护）。

### 8.2.2 UBX 解析器未消费第二同步字节（RX 侧同类 bug，2026-10-23）

修正发送侧 CK_B 后依然 `UBX@9600=NO`，但新增埋点直接拍到了模块的应答：

```
W gnss: probe @9600 fail: rx=198 B, sync+1, nmea+0, first: B5 62 0A 04 BE 00 32 2E ...
```

`B5 62 | 0A 04 | BE 00` 就是 MON-VER，长 190 字节，整帧 198 B 全部收到 —— 说明 TX/RX 都正常，**是解析器丢了它**。

原因：`feed_byte()` 进入 UBX 模式后 `s_ubx_pos = 0` 就当 class 收，**没消费第二个同步字节 0x62**，于是
`class=0x62, id=0x0A, lenLo=0x04, lenHi=0xBE → len=0xB804=47108 > 512` → 整帧丢弃。
后果：所有 UBX 应答（MON-VER/ACK/NAV-PVT/CFG 回读）永远解析不出来，而 NMEA 走的是行解析，完全不受影响。

修复：把 pos 语义改为 `0=0x62、1=class、2=id、3=lenLo、4=lenHi、payload 自 5 开始`，并校验 pos0 必须是 0x62，否则重新找 `B5`。
新增 `ubx_parse_selfcheck()`：启动时喂一个合成 ACK 帧走真实 `feed_byte()` 路径，必须能解析出 `05/01 len=2`；
完成后快照/还原全部解析状态（含 `s_last_frame_ms`、`s_last_rx_ms`，避免干扰波特率锁定判定）。
`COUNTERS` 日志增加 `UBX frame=` 与 `sync=` 两个计数，便于现场判别“没收到”还是“收到了解析不了”。

### 8.3 排障开关 `GNSS_DISABLE_FOR_UCENTER`

> **背景**：u-center 抓包显示模块 1 Hz 稳定输出但 **`numSV=0`、RMC 状态 `V`、GGA 质量 0**（GSV 仅 18 B = 一句零星报文），同时出现 `UNKNOWN` 碎帧与 `UBX MON Size 8`（MON-VER poll 无应答）。怀疑固件双主机抢线/配置干扰，需做“纯净链路”对照实验。

`config.h` 中 `#define GNSS_DISABLE_FOR_UCENTER`（当前已回 `0`；排障时置 `1`）为排障模式，替代 `gnss_init()` 调用 `gnss_power_only_init()`：

| 项目 | 正常模式（0） | 排障模式（1） |
| --- | --- | --- |
| GNSS LDO EN（GPIO14） | 拉高 | **仍拉高**（模块必须带电，否则 u-center 无数据） |
| UART1 驱动 | `uart_driver_install` | **不安装** |
| TX17 / RX18 | UART 功能 | **高阻输入**（不驱动模块 RX，不与 u-center 抢总线） |
| `gnss_task` | 创建 | **不创建** |
| 波特率探测 / SELFTEST / PUBX,41 / UBX 配置与轮询 | 全开 | **一字节不发** |
| 各 getter | 真实快照 | 互斥锁照建，返回全零快照（不崩、UI 显示 0 星） |

实验结论（已归档）：
- 释放 UART 后 **MON-VER 立即有应答、`UNKNOWN` 碎帧消失** → 之前那份日志里的“MON-VER 无应答 + UNKNOWN”确实是固件与 u-center 双主机抢线造成。
- 模块身份已由 GNTXT banner 确认：**正品 NEO-M8N-0 / SPG 2.01 / PROTVER 15.00**（`FIS found`、`LLC` 行、`MOD NEO-M8N-0` 均在），此前“克隆件”推测已作废。
- 星座配置失败的真实原因是 **BDS 被 OTP 禁用 + CFG-GNSS 写法**，与抢线无关，详见 §8.2。

> 约束：该开关**仅用于排障**，验证完必须改回 `0`；禁止在发布固件中置 1。

> **约束 D-10（解析正确性）**：
> - NMEA 行缓冲 ≥128 B；UART RX ring buffer ≥ 2× 最大句长；**禁止在中断里做字符串解析**。
> - **GSV 多句累积**：按 total/sentence index + talker 分别累积，卫星数组上限 32，跨 talker 清空重开。
> - 校验和失败 / RMC 无效（`A` 标志缺失）/ 无 fix 的时间一律丢弃。
> - 配置命令独立等待 ACK，**超时 ≤500 ms**，失败重试 1 次后降级（保持上次可用配置），禁止阻塞式死等。
> - 双协议只采纳有 ACK 的一方；PMTK 用 `$PMTK001` 应答确认。

### 8.4 最终 GNSS 配置基线（2026-10-23 实测通过）

实物：NEO-M8N-0 / SPG 2.01 / PROTVER 15.00，OTP 仅授权 GPS+GLONASS。

| 项 | 下发值 | 依据 |
| --- | --- | --- |
| 串口 | UART1 @ **115200**，8N1，`mode=0x000008C0` | 回读值一致；9600 容量不够 10Hz 多消息 |
| `CFG-PRT` 协议 | inProto=`0x0007`（UBX+NMEA+RTCM2），outProto=`0x0003` | 保留 RTCM2 输入位，便于以后 RTK |
| `CFG-RATE` | meas=100 ms，navRate=1，timeRef=GPS | **随星座联动**：单 GPS → 10 Hz（100 ms）；GPS+GLO → 5 Hz（200 ms） |
| `CFG-MSG`(UART1) | NAV-PVT=1（10 Hz）；NAV-SAT=5、NMEA GGA/RMC/GSA/GSV=5（2 Hz） | 带宽估算 ≈1.6 kB/s，115200 余量充足 |
| `CFG-GNSS` | 5 块全列（GPS 8/16、SBAS 1/3、BDS 8/16、QZSS 0/3、GLO 8/14），**只改 enable 位**：GPS+GLO 开，SBAS/BDS/QZSS 关 | 只发 2 块会被规范化；BDS 被 NAK |
| `CFG-NAV5` | dynModel=0(portable)，fixMode=3(auto 2D/3D) | 骑行场景够用 |
| `CFG-CFG` | saveMask=0x03DF，deviceMask=0x03（BBR+Flash） | 掉电后仍保持 115200/10Hz |
| 不动 | `CFG-ANT`（VCC_RF 未焊）、`CFG-PM2`（省电关闭）、`CFG-ODO`（关闭） | 保持默认即可 |

星座结论：**GPS + GLONASS 可用**；BDS 请求必 NAK（OTP 未授权，已由 `s_bds_blocked` 锁存，不再重复尝试）；
SBAS/QZSS 已按模板关闭以把通道让给 GPS/GLO。Galileo 在本固件能力串中不存在。

更新率约束（u-blox 8 并发星座降额，`gnss_rate_limit()` 实现）：

| 星座组合 | 上限 | 设置页选项（已去掉 18 HZ） |
| --- | --- | --- |
| 仅 GPS | **10 Hz**（100 ms） | 10 / 5 / 1 HZ |
| GPS+GLONASS（含 GPS+BDS 目标被降级后） | **5 Hz**（200 ms） | 选 10 HZ 会自动压到 5 Hz 并打印 WARN |
| 18 Hz | 本模块不支持 | 已从 UI 移除（`gnss_set_update_rate()` 仍兼容旧 NVS 值，按 10 Hz 处理） |

实现：`cfg_start()` 按下发前就 clamp；`cfg_finish()` 在星座确定后再校一次（BDS NAK 降级为 GPS+GLO 后自动从 10 Hz 降到 5 Hz 并重发 CFG-RATE/MSG）。
状态栏显示的是 `gnss_data_t.rate_hz`（**实际生效值**），不是设置项里的选择值。

---

## 9. 传感器与校准

### 9.0 安装方向与轴向约定（2026-08-06 用户确认）

- **安装方式**：设备竖置（长边竖直），屏幕正对用户，传感器贴装于 PCB 背面（Bottom Layer）。
- **设备坐标**：X = 屏幕平面向右；Y = 竖直向上（长边机身顶部）；Z = 屏幕法线朝向用户。
- **PCB 背面贴装右手系映射**：
  - IMU（LSM6DSR）：X_dev = -X_chip, Y_dev = +Y_chip, Z_dev = -Z_chip（`IMU_AXIS_X_SIGN=-1, Y=1, Z=-1`）；
  - 磁力计（LIS2MDL）：逆时针 90° 贴装，X_dev = -m1(OUTY), Y_dev = -m0(OUTX), Z_dev = -m2(OUTZ)；
- **全姿态自适应 3D 倾角补偿与地磁偏角自动修正**：平放时以机身顶部为基准，竖直立起时由 IMU 重力自适应平滑过渡至机身背面视线，GPS 获得有效经纬度后自动计算当地 WMM 地磁偏角并注入真北航向。

- `sensors_init()`：初始化 I2C/ADC 与充电状态 GPIO；依次唤醒 LSM6DSR、LIS2MDL、BMP388；校验芯片 ID（见 README C-02）；校准数据从 NVS `cal` 读取，缺省用零偏 + 单位软铁系数。
- `sensors_update()`：IMU 应用轴向翻转 + 偏移；磁力计按芯片贴片重映射至机身右手坐标系（X=-OUTY, Y=-OUTX, Z=-OUTZ），并应用物理通道硬铁/软铁系数缩放；**随后调用 `ahrs_update()` 做九轴四元数姿态融合**（x-io Fusion v1.3.3，ENU，含加计/磁干扰拒绝），航向优先取四元数解算结果，未收敛时回退到静态 3D 倾角补偿；BMP388 官方补偿公式；电源用 oneshot ADC + curve fitting + 丢弃前 2 次/16 次采样/中间 50% 平均 + `BAT_ADC_CAL_GAIN/OFFSET` 单点校准（饱和保护见 README §3.4）。
- `ahrs.c` 适配层：单位换算（mdps→dps、mg→g、mGauss→uT）、上电解析初始四元数（跳过 Fusion 3 秒启动收敛）、平放/竖立重力加权前向视线，输出 `quat[4]` / `roll_deg` / `pitch_deg` / `ahrs_heading_deg` 与磁干扰、加计拒绝标志。

> **约束 D-13（第三方融合库）**：姿态融合算法不得自行重写，统一使用 `components/fusion/`（x-io Technologies Fusion，MIT，固定 tag v1.3.3）；适配层 `main/ahrs.c` 只做坐标系/单位转换，升级库版本时只改 `components/fusion/`。
- `sensors_start_calibration()` 后台 FreeRTOS 任务：IMU 采 512 个静止样本求均值；磁力计采 600 个 8 字摇晃样本求偏置 + 软铁系数；进度/提示通过 `sensors_calibration_status_t` 暴露给 UI；完成后写 NVS `cal`。

> **约束 D-11（校准状态机）**：校准期间禁止切换模式/停止采集；进度必须单调；UI 提示 “保持静止” / “8 字晃动” 与百分比实时同步；校准完成写 NVS 成功后才允许退出。

---

## 10. 设置菜单与持久化（9 大配置项契约与自动同步机制）

- `settings_store`：NVS 保存 9 项系统配置；`app_main` 启动时同步到 `system_context_t` 并立即应用。
- 9 项配置条目及其动作规范：
  1. **FUNCTION MODE**：`P-GEAR` (0) / `TRACK REC` (1) / `BIKE COMP` (2)；波轮轮转编辑，短按确认，实时切换主屏；
  2. **COLOR THEME**：`SUNLIGHT` (0) / `DARK` (1)；短按循环步进，全局色表与样式切换；
  3. **GPS RATE**：`10 HZ` / `18 HZ` / `1 HZ` / `5 HZ`；`PMTK220` + `UBX-CFG-RATE`，ACK 成功才写 NVS；
  4. **GNSS MODE**：`GPS+BDS` / `ALL GNSS` / `GPS ONLY`；`PMTK353` + `UBX-CFG-GNSS`，日志输出 ACK/NAK；
  5. **BRIGHTNESS**：`50%` (0) / `75%` (1) / `100%` (2) / `25%` (3)；与底层默认基线 50% 严格一致，动态调节 GPIO9 LEDC PWM 占空比并落盘；
  6. **AUTO SLEEP**：`3 MIN` / `5 MIN` / `NEVER` / `1 MIN`；无操作自动进入低功耗待机；
  7. **RTC AUTO SYNC**：`ON` (默认) / `OFF`；
     - **契约**：当处于 ON 时，GNSS 首次输出有效 3D Fix（RMC/ZDA 语句合法）后，立即提取 UTC 年月日时分秒，调用 `settimeofday()` 校准系统硬件时钟，并在状态栏时钟显示真实时间；
  8. **ALT AUTO CALIB**：`ON` (默认) / `OFF`；
     - **契约**：当处于 ON 时，开机后持续监听 GNSS 质量。在同时满足以下条件时执行**单次气压高度基准反算**：
       - a. GNSS 处于稳定 3D 定位状态（`fix_type >= 2`）；
       - b. 垂直精度置信度高（高度误差估计值 `< 5.0m`，且 `VDOP ≤ 2.5`）；
       - c. 气压计数据有效且平稳（最近 5 个采样方差极小）；
     - **执行动作**：以 GNSS 当前绝对椭球/大地高为准，依据国际标准大气公式反算当前海平面参考基准气压 $P_0$（QNH）或高度零偏 offset，修正 BMP388 测量基准；
     - **单次锁存保护**：置位全局 `s_alt_calibrated_this_boot = true`，**每次开机仅自动校准一次**，后续即使卫星跳变或进入建筑阴影也绝不重复校准，确保气压计相对高程测量的极佳连续性；
  9. **SENSOR CALIB**：短按启动真实传感器物理校准引擎（**坚决杜绝假进度条**）：
     - a. **IMU 校准前 5 秒准备倒计时**：界面大字显示 5s 倒计时，提示“请将设备水平平放好”；
     - b. **IMU 真实零偏校准（策略 A：振动清零）**：设备必须保持严格水平静止（三轴陀螺角速度严格 < 3.5 dps，加速度合力严格在 0.94G ~ 1.06G 之间）。**一旦检测到手抖、振动或晃动，执行策略 A：累积计数立即清零，进度条瞬间打回 0%**！只有在完全无振动台面上连续平稳静止满 2 秒（40 帧连续采样），进度条才线性涨至 100% 并计算真实零漂；
     - c. **地磁校准前 5 秒准备倒计时**：界面大字显示 5s 倒计时，提示“请拿起设备准备在空中做 8 字晃动”；
     - d. **地磁空中 8 字真实校准（30 秒超时保护）**：用户在空间中做“8 字晃动”，系统追踪动态三维极值跨度与 8 个空间象限覆盖度；**若设备静止平放，进度条绝对恒为 0% 绝不上涨**！只有真实覆盖 ≥7 个空间象限且动态范围充足后才算 100% 收敛；**若在 30 秒内未完成 8 字翻转，触发超时事件**，界面红字提示“CALIBRATION TIMEOUT! PLEASE RE-CALIBRATE”，不保存未收敛数据；
     - e. **完成判定**：只有在 8 字校准在 30 秒内完全达标收敛后，状态机才置位 `CALIB_PHASE_DONE`，界面居中呈现显著的 **“校准 OK”**（`[OK] CALIBRATION SUCCESS`）字样与 **“保存退出”**（`SHORT: SAVE & EXIT`）提示；此时短按按键将校准参数落盘 NVS `cal` 分区，在运行时立即生效；
     - f. **安全放弃**：在以上任意阶段中，用户随时可通过长按（≥800ms）终止并丢弃未完成数据，安全回退上级。
- **全量设置与校准 NVS 持久化架构**：
  - 系统设置（功能模式、主题、刷新率、星座、亮度、休眠、RTC、高度校准）在用户修改时立即落盘至 NVS `"settings"` 分区；
  - 传感器校准数据（陀螺零漂、加速度偏置、磁力计硬铁偏移与软铁缩放矩阵）在校准完成后立即落盘至 NVS `"cal"` 分区；
  - 开机时系统优先从 Flash 载入，绝不上电重置！
- `diagnostics_trigger_event()` 在每次配置下发与校准落盘后打印详细事件日志。

### 10.1 自动同步与校准后台工作流 (Mermaid)

```mermaid
flowchart TD
    Start([开机启动 / GNSS 初始化]) --> Monitor[后台任务监听 GNSS 质量快照]

    Monitor --> CheckRTC{"RTC AUTO SYNC == ON ?"}
    CheckRTC -- 是 --> CheckGPSFix{"GNSS 3D 定位有效 ?<br>(卫星数 ≥ 4 且时间有效)"}
    CheckGPSFix -- 是 --> DoSync["提取 UTC 年月日时分秒<br>调用 settimeofday 同步 RTC"]
    DoSync --> RTC_Done([RTC 同步完成 / 状态栏生效])
    CheckGPSFix -- 否 --> WaitGPS1[等待下一次定位]
    CheckRTC -- 否 --> SkipRTC[跳过 RTC 同步]

    Monitor --> CheckAltSwitch{"ALT AUTO CALIB == ON ?"}
    CheckAltSwitch -- 是 --> CheckCalibOnce{"本次开机已校准过 ?"}
    CheckCalibOnce -- 否 --> CheckQuality{"GNSS 3D 定位良好 ?<br>(VDOP ≤ 2.5 且误差小于 5m)"}
    CheckQuality -- 是 --> DoAltCalib["以 GNSS 高度为绝对基准<br>反算海平面基准气压 P0<br>校准气压计高度"]
    DoAltCalib --> LockCalib["置位: alt_calibrated_this_boot = true<br>(开机锁存，后续不重复校准)"]
    LockCalib --> Alt_Done([气压计高度校准完成])
    CheckQuality -- 否 --> WaitGPS2[等待高精度条件]
    CheckCalibOnce -- 是 --> SkipAlt[开机已校准，保持锁定]
    CheckAltSwitch -- 否 --> SkipAltCalib[跳过自动校准]
```

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
| 3b | **GPIO11=WATCHDOG（Q4 相关），暂不开发**；**Q3=LCD 背光驱动**（用户确认） | GPIO11 固件不驱动（保持输入/不初始化）；LCD 背光 = GPIO9 PWM → Q3（DTC123JCA）驱动背光 LED，**Q3 与看门狗无关**；Q4 接 WATCHDOG 网络，电路逻辑待确认 |
| 4 | **GPIO12=ADC2_CH1 且 1:1 分压 4.2 V 超量程** | 核实分压比；固件饱和保护；将来开 Wi-Fi 必须迁 ADC1 |
| 5 | **GNSS 已定 NEO-M8N-0-01（UBX/9600）；ATGM336H（PMTK/9600）位号备选** | 默认 UBX 协议栈；双协议下发只认 ACK 方；波特率探测 [9600→38400→115200]（README §3.5） |
| 5b | **替代料已定：LSM6DSRTR（原 LSM6DSVETR）、BMP388（原 BMP390L）、NEO-M8N-0-01（原 NEO-M9N-00B）** | 三组均为封装+引脚完全兼容；芯片 ID 宽松校验：IMU=0x6B（勿写 0x6A）、磁力计=0x40、气压计=0x50；**NEO-M8N 最多 3 星座并发**，4 星座会 NAK → 降级 GPS+GLONASS+BeiDou |
| 6 | **GSV 多句漏累积** | 按 total/index + talker 累积，上限 32，跨 talker 重置 |
| 7 | **NMEA 年份 2 位 → 时间戳错误** | 80 年滚动窗口（2000~2079），无效 fix 时间丢弃 |
| 8 | **LVGL 跨任务调用崩溃** | 全部 LVGL 调用锁在 `ui_task` |
| 9 | **等待 ACK 无超时 → 死等** | 所有等待 ≤500 ms 超时 + 重试 1 次 + 降级 |
| 10 | **高优先级持锁低速 I/O → 优先级反转** | 按 D-01，锁内禁止低速 I/O |
| 11 | **两处累计距离/时间 → 数据不一致** | 单份累计状态（§7），写文件与显示同一来源 |
| 12 | **ADC 未校准读值非线性** | oneshot + line fitting 校准；禁止直接读原始值 |
| 13 | **I²C 1 MHz 无外部上拉不稳** | 确认上拉；调试期降 400 kHz |
| 14 | **NVS 频繁写磨损** | 值变化才写；配置 ACK 成功才落盘 |
| 15 | **IDF/LVGL 版本漂移** | CI 锁定 release-v6.1 + LVGL 9.5.0 + esp_lvgl_port 2.8.0，`dependencies.lock` 入库；LVGL 9 与 8.x API 不兼容（lv_display_*/lv_screen_active/新 flush_cb），禁止按旧教程写 v8 代码 |

---

## 13. 后续扩展建议

1. **轨迹可视化**：GPS 记录界面叠加折线轨迹或缩略地图，提升现场复盘能力（注意 UI 刷新频率约束 D-06）。
2. **传感器融合**：Kalman 融合 IMU+GNSS 推算姿态/坡度，提升 P-Box 与骑行分析精度。
3. **连接能力**：BLE/Wi-Fi/USB CDC 传输 GPX 或实时数据 + OTA；注意 GPIO19/20 已预留、Wi-Fi 开启后 ADC2 不可用（README §3.4）。
