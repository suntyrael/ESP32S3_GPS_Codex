# ESP32-S3 多功能 GPS 性能分析设备

## 1. 项目概述

基于 **ESP32-S3FH4R2**（4 MB Flash / 2 MB PSRAM，均封装在片内）的自研固件，集**自行车码表、GPS 轨迹记录仪、汽车 P-Box 性能测试盒、GNSS 信息诊断、设置与校准**于一体。系统围绕 FreeRTOS 与 LVGL v8.3 构建，全部硬件驱动、传感器融合、数据存储与诊断日志自研实现。

- **固件版本**：V0.0.1（release 提供合并烧录包）
- **开发环境**：ESP-IDF v6.1.0
- **目标平台**：ESP32-S3FH4R2（仅此一种，禁止交叉编译到其他芯片）

---

## 2. 版本锁定（禁止漂移）

| 组件 | 锁定版本 | 说明 |
| --- | --- | --- |
| 芯片 | ESP32-S3FH4R2 | 4 MB Flash + 2 MB PSRAM（封装内） |
| ESP-IDF | **v6.1.0** | 禁止使用 master；升级需单独评审 |
| LVGL | **8.3.11**（`"~8.3.0"`） | **禁止升级 v9**（API 完全不兼容） |
| 组件来源 | components.espressif.com | 通过 `main/idf_component.yml` 声明 |
| 工具链 | IDF v6.1 自带 | `idf.py` ≥ 5.x 命令行 |

> **约束 C-01**：`dependencies.lock` 必须入库；CI 与本地构建使用完全相同的版本组合，禁止手动改锁文件。

---

## 3. 硬件规格

### 3.1 关键器件

| 器件 | 型号（实际贴装 / 原理图位号） | 总线/地址 | 特性 |
| --- | --- | --- | --- |
| GNSS | **NEO-M8N-0-01**（U39，实际贴装；U3/ATGM336H-F8N76 位号备选，未贴） | UART1 @115200（上电默认 9600，需探测） | **默认 UBX 协议栈**；NEO-M8N 最多 3 星座并发 |
| IMU | **LSM6DSRTR**（U9；替代 LSM6DSVETR，LGA-14L 引脚完全一致） | I²C 0x6A/0x6B（SA0 决定） | 陀螺+加速度；轴向修正：**Z 反向，Y 不变**；WHO_AM_I=0x6B |
| 磁力计 | LIS2MDLTR（U10） | I²C 0x1E（固定地址） | 轴向修正：**X 正常，Y 交换且反向，Z 反向** |
| 气压计 | **BMP388**（U37；替代 BMP390L，LGA-10 引脚一致） | I²C 0x76（SDO=0） | 官方补偿公式；CHIP_ID=0x50 |
| 显示屏 | ST7789 240×320（U2） | SPI3 + DMA 双缓冲 | 竖屏、旋转 180°；**背光 = GPIO9 PWM → Q3（DTC123JCA）驱动** |
| 存储 | microSD（CARD1）4-bit SDIO | SDMMC（GPIO matrix 路由） | GPX 存 `/GPX/` |
| 充电 | TP4054（U34） | — | CHRG 引脚 → CHG_SAT |
| 电源 | SGM61020（U36）DC-DC | — | VBAT→3.3 V（L1 2.2 µH） |

> **约束 C-02（芯片 ID 校验，宽松校验）**：启动时读取器件 ID 并与**候选列表**比对，命中任一项即通过；未知值打日志降级，**禁止**用单一硬编码值拒绝启动：
> - IMU（LSM6DSV/LSM6DSR 系）：`WHO_AM_I = 0x6B`（LSM6DSR 规格书 p50 明确；旧文档写 0x6A 系误，勿作唯一判据）
> - 磁力计 LIS2MDL：`WHO_AM_I = 0x40`
> - 气压计：`CHIP_ID = 0x50`（BMP388）/ `0x60`（BMP390，板载型号，两者都接受）

### 3.2 引脚分配表

| 功能 | GPIO | 备注 |
| --- | --- | --- |
| DEBUG_TX / DEBUG_RX | 43 / 44 | UART0 @115200，调试日志专用 |
| DISP_SCK / MOSI / CS / DC / RST / BL | 5 / 8 / 7 / 6 / 4 / 9 | SPI3，BL 2 kHz PWM 默认 50% |
| GNSS_TX / GNSS_RX / GPS_LDO_EN | 17 / 18 / 14 | UART1（默认引脚）；LDO 高电平使能 |
| I2C_SCL / I2C_SDA | 39 / 40 | I2C0 @1 MHz |
| ACCGYRO_INT / MAG_INT / PRESS_INT | 41 / 42 / 13 | **当前未使用，禁止被其他外设占用** |
| SD_CLK / SD_CMD / SD_D0 / SD_D1 / SD_D2 / SD_D3 | 36 / 35 / 37 / 38 / 34 / 33 | 4-bit SDIO（原理图已核实） |
| ENC_A / ENC_B / KEY_MAIN（原理图：SWA / SWC / PUSH） | 1 / 3 / 2 | 编码器 A/B 上拉；主按键上拉 |
| DL_KEY（下载键） / GPIO10 / WATCHDOG | 0 / 10 / 11 | GPIO0 经 499 Ω 接下载键；**GPIO10 空闲**（未接 CHIP_PU）；GPIO11 接 WATCHDOG 网络（Q4 相关，**暂不开发**） |
| BAT_ADC / CHG_SAT | 12 / 21 | ADC2_CH1；充电状态输入（原理图命名 CHG_SAT） |
| GPIO15 / 16 | 空闲 | 未接，禁止分配外设 |
| XTAL_32K_P/N | — | 板载 32.768 kHz 晶振（C30/C31 12 pF） |

> ✅ **SD 映射已按原理图核实**：CLK=36、CMD=35、D0=37、D1=38、D2=34、D3=33。GPIO33~38 在本芯片（四线 PSRAM）下可用，与 Flash/PSRAM 无冲突（若误配 Octal PSRAM 则冲突，见 §3.3-2）。**旧文档中的矛盾值（CMD=35 与 D2=35 冲突等）作废，编码一律以本表为准**。

### 3.3 引脚可用性约束（编码前必读）

1. **GPIO26~32 禁用**：ESP32-S3FH4R2 的封装内 Flash/PSRAM 通过 SPI0/1 占用这些引脚（SPICS0/1、SPICLK、SPID0~3 共 7 根），任何使用都会导致编译或运行异常。
2. **GPIO33~37 是否可用取决于 PSRAM 的 SPI 模式**（乐鑫官方规则）：
   - **四线（Quad）模式**（本项目 FH4R2 采用）：GPIO33~37 **可用**，SD 卡已占用 GPIO33/34/36/37/38 + CMD=35，无冲突；
   - **八线（Octal）模式**（如 FH4R8 或个别批次芯片）：GPIO33~37 被额外占用，**SD 卡方案必须重做**。
   - **对应构建约束**：`CONFIG_SPIRAM_MODE_QUAD=y`，**禁止**配置为 OCT（否则 SDK 会去占用 GPIO33~37，与 SD 冲突）。
3. **Strapping 引脚 = GPIO0 / GPIO3 / GPIO45 / GPIO46**（规格书表 3-1：GPIO0 弱上拉=1、GPIO3 浮空、GPIO45/46 弱下拉=0）：本设计用到 **GPIO3（ENC_A/SWC）**，复位默认**浮空**，靠外部上拉保证为高（JTAG 信号源默认值）；**编码时禁止改变其复位时序**；GPIO45 影响 VDD_SPI 电压（1.8/3.3 V），禁止改动。GPIO0 已接下载键（DL_KEY），默认高=SPI boot。
4. **GPIO19 / 20 = USB-C（DP/DN）**：板载 USB-C 座，芯片复位默认开启 USB 功能并带 USB 上拉；固件可选用 USB-CDC/下载，**禁止改作普通 GPIO 外设**。
5. **GPIO43 / 44 禁用**：UART0 调试口。
6. **GPIO22~25 芯片上不存在**，禁止出现在任何配置中。
7. 全部外设引脚通过 **GPIO matrix** 配置（非默认 IO MUX 的引脚必须先 `gpio_*` 初始化再使用外设驱动）。

### 3.4 硬件注意事项（踩坑预防）

- **BAT_ADC = GPIO12 = ADC2_CH1**（注意：ESP32-S3 的 ADC1=GPIO1~10、ADC2=GPIO11~20，与原 ESP32 不同）。**若将来启用 Wi-Fi，ADC2 与 Wi-Fi 冲突，电池采样必须迁移**。
- **ADC 量程**：11 dB 衰减 + 校准后的有效量程约 0.1~3.1 V。1:1 分压下满电单节锂电（4.2 V）会饱和。**编码前核实分压比**；固件必须做饱和保护（超量程按满格处理并打日志），禁止信任未校准的 ADC 原始值。
- **I²C 1 MHz**：需确认板上有外部上拉；若无外部上拉，内部上拉（约 45 kΩ）可能不够，调试时先降速 400 kHz。
- GNSS 双协议差异：**实际贴装 NEO-M8N-0-01（U39），默认走 UBX 协议栈**；U3 位号（ATGM336H-F8N76）未贴装，PMTK 通道仅作兼容保留（检测到 PMTK 应答才启用）。**波特率**：NEO-M8N 与 ATGM336H 上电默认都是 9600 → 启动仍做速率探测 [9600 → 38400 → 115200]（见 dev_note §8）。
- **SD 走 SDMMC**：ESP32-S3 的 SDMMC 信号经 GPIO matrix 路由，可用任意引脚，但需在 `sdmmc_host_t` 中显式配置 `slot` 与引脚；4-bit 模式必须 6 根信号（CLK/CMD/D0~D3）全部正确。CMD/D0~D3 已有 10 kΩ 上拉（R23~R27）。

### 3.5 原理图与规格书核对记录（2026-08）

**已逐脚核实的 GPIO 分配**（原理图 `DOC/SCH_Main_2026-08-06.pdf` 坐标级验证，全部与 §3.2 一致）：

- SDIO：CLK=36 / CMD=35 / D0=37 / D1=38 / D2=34 / D3=33
- I2C0：SCL=39 / SDA=40；INT：ACCGYRO=41 / MAG=42 / PRESS=13
- LCD：RST=4 / SCK=5 / DC=6 / CS=7 / SDA(MOSI)=8 / BL=9
- GNSS：TX=17 / RX=18 / EN=14；UART0=43/44；USB=19/20
- 特殊：GPIO0=DL_KEY、**GPIO10=空闲（未接 CHIP_PU，用户确认）**、GPIO11=WATCHDOG（暂不开发）、GPIO15/16 空闲

**规格书要点（与旧文档差异）**：

1. ESP32-S3 表 2-14：Quad SPI 模式 SPI0/1 占用引脚 28~35（GPIO26~32），GPIO33~37 在 Quad 下**不占用**（仅 Octal 模式占用为 DQ4~DQ7/DQS）→ 四线 PSRAM 结论成立。
2. Strapping 默认值（表 3-1）：GPIO0=弱上拉 1、GPIO3=浮空、GPIO45=弱下拉 0、GPIO46=弱下拉 0。
3. ADC（表 2-8）：GPIO1~10=ADC1_CH0~9，GPIO11~20=ADC2_CH0~9 → GPIO12=ADC2_CH1。
4. 默认引脚：U0TXD/RXD=43/44、U1TXD/RXD=17/18；GPIO19/20 复位默认 USB 功能。
5. LSM6DSR 规格书：WHO_AM_I=0x6B；LIS2MDL：WHO_AM_I=0x40（I²C 地址固定 0x1E）；BMP388：CHIP_ID=0x50、I²C 0x76(SDO=0)/0x77(SDO=1)。
6. 替代料确认（封装/引脚完全兼容）：LSM6DSVETR→LSM6DSRTR、BMP390L→BMP388、NEO-M9N-00B→NEO-M8N-0-01；NEO-M8N 默认 9600 波特、最多 3 星座并发。

**待办（open items）**：

- [x] IMU 实际贴装 **LSM6DSRTR**（替代 LSM6DSVETR）→ 规格书已上传：WHO_AM_I=0x6B，LGA-14L 引脚与 LSM6DSV 逐脚一致
- [x] 气压计实际贴装 **BMP388**（替代 BMP390L）→ 规格书已上传：CHIP_ID=0x50，LGA-10 引脚一致
- [x] GNSS 实际贴装 **U39 = NEO-M8N-0-01**（替代 NEO-M9N-00B）→ **UBX 协议栈为默认**；默认 9600 波特；最多 3 星座并发（4 星座会 NAK）
- [x] GPIO10 **未连接 CHIP_PU**（用户确认）→ 为空闲引脚，禁止分配外设
- [ ] WATCHDOG（GPIO11 / Q4）**暂不开发**（用户决定）：固件不驱动该引脚，待 Q4 电路逻辑确认后再启用；**Q3 已确认为 LCD 背光驱动**（LCD_BL PWM → Q3），与看门狗无关

---

## 4. 功能规格

### 4.1 模式总览与切换

旋转编码器依次循环：`自行车码表 → GPS 记录 → P-Box → GNSS 信息 → 设置 → …`。

| 操作 | 效果 |
| --- | --- |
| 短按 | 确认/执行（P-Box 模式切换 READY↔ARMED/FINISHED） |
| 中按（~500 ms） | 开始/停止轨迹记录 |
| 长按（~2000 ms） | 进入/退出设置界面 |
| 双击（≤400 ms） | 预留应用层扩展 |

### 4.2 自行车码表（MODE_BIKE_COMPUTER）

- 48 px 速度大字显示；海拔 / 累计里程 / 骑行时间分区块。
- 录制状态指示：未记录=绿色圆圈，记录中=红色闪烁方块。

### 4.3 GPS 轨迹记录仪（MODE_GPS_LOGGER）

- 速度（100 px）/ 轨迹图（120 px）/ 距离（40 px）/ 时间（40 px）。
- 长按控制录制；状态栏显示 GPS / SD / 电池。

### 4.4 P-Box 性能测试（MODE_PBOX）

- 64 px 速度、32 px 计时、目标区间与状态提示。
- **启动条件**：GPS 速度 < 1 km/h **且** IMU X 轴线性加速度 > 0.15 G（阈值 `config.h` / 设置菜单可调，0.10~0.30 G）。
- 测试完成显示 “TEST FINISHED!!!”。

### 4.5 GNSS 信息界面（MODE_GNSS_INFO）

- 经纬度、海拔、速度；HDOP/VDOP/PDOP。
- 卫星列表（ID、星座、CN0、状态、可滚动）：搜索=灰、跟踪=黄、使用=绿。

### 4.6 设置菜单（MODE_SETTINGS）

- IMU 校准、磁力计校准、GNSS 刷新率（1/5/10/25 Hz）、GNSS 动态模式（步行/汽车/海上/航空）、星座组合、P-Box 阈值、显示亮度、关于/版本。
- 校准在后台任务执行，UI 实时显示提示与百分比进度，完成后写 NVS。

---

## 5. 软件架构总览

详细任务拓扑、数据结构契约、状态机与数据流见 **dev_note.md**。要点：

- FreeRTOS 多任务：`sensor_task` / `ui_task` / `input_task` / `gpx_task` / `diagnostic_task` + `input_manager` 内部任务。
- 硬件状态统一由 `sensors_get_state()` 提供**拷贝快照**，所有任务只读快照，禁止直接访问硬件缓冲。
- 所有可持久化配置收敛在 `config.h` + NVS（`settings_store` / `cal` 两个命名空间）。

---

## 6. 数据与协议规格

### 6.1 NMEA（GNSS）

- 解析 `GGA / RMC / GSA / GSV`，按 talker 识别星座：`$GPGSV`=GPS、`$GLGSV`=GLONASS、`$GAGSV`=Galileo、`$BDGSV`=BeiDou。
- 校验和：`*XX`（`$` 与 `*` 之间异或，只接受 0x00~0x7F）。
- **GSV 必须支持多句累积**（total/sentence index），且按 talker 分别累积，禁止跨 talker 串句。
- 卫星数组上限 32 颗；CN0 着色：>45 绿、35~45 黄绿、25~35 黄、<25 橙/红。

### 6.2 UBX / PMTK 配置（并行下发）

- 所有刷新率、星座组合、动态模式设置**同时**下发 PMTK 与 UBX 命令：`PMTK251/220/353`、`UBX-CFG-RATE/GNSS/NAV5`。
- **NEO-M8N 星座上限**：最多同时启用 3 个星座；`UBX-CFG-GNSS` 请求 4 星座返回 NAK 时，降级为 GPS+GLONASS+BeiDou 并记日志。
- 每条配置命令必须等待对应 **ACK/NAK**：UBX 按 (class, msgID) 匹配 ACK-ACK/ACK-NAK；PMTK 以 `$PMTK001` 应答。
- ACK/NAK 结果写入诊断日志；**等待超时 ≤ 500 ms**，失败重试 1 次后降级（见 dev_note §8）。

### 6.3 GPX 记录格式

- 文件名 `/GPX/ACT_xxxx.gpx`（4 位序号）。
- `<trkpt>` 含坐标、海拔、`<time>`；`<extensions>` 含温度、三轴/总 G、气压、电池百分比、电压、运行模式、P-Box 上下文、骑行/轨迹距离、P-Box 计时。
- **时间一律 UTC + ISO8601**（如 `2026-08-05T06:14:00Z`）。
- 写盘策略：事件队列驱动独立 `gpx_task`，每个样本 `fflush`，STOP 时闭合标签并关闭文件。

### 6.4 时间与 RTC

- 首次启动用固件编译时间；GNSS 获得有效时间后自动同步 RTC，并在屏幕中央显示 “时间已同步” 2 s。
- 全部时间以 **UTC** 存储与记录；NMEA 2 位年份按 **80 年滚动窗口**（2000~2079）转换；无有效 fix 或校验失败的时间一律拒绝。

---

## 7. UI 规格（240×320 竖屏）

- **状态栏 20 px**：GPS 图标 + 卫星数（红=未定位，绿=定位）、SD 图标、电池百分比 + 充电图标。
- **字体**：Small 8 / Medium 12 / Large 16 / XL 24 / 大数字 48~64 px（大数字字体只包含数字与必要符号以省 Flash）。
- **间距**：小 5 / 中 10 / 大 15 px。
- **颜色**：主文字白、次文字灰、背景黑；记录中红色闪烁；警告橙。
- 全部布局常量集中在 `ui_common.h`，禁止 UI 层出现魔数。

---

## 8. 构建与烧录

### 8.1 环境

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash          # 或 release 的合并镜像
```

### 8.2 sdkconfig.defaults 必配项（入库，保证可复现构建）

```ini
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_SPIRAM=y                       # 2 MB PSRAM
CONFIG_SPIRAM_MODE_QUAD=y             # 四线 PSRAM，禁止改 OCT（否则占用 GPIO33~37，与 SD 冲突）
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y  # 单 App 大分区，预留 ≥30% 余量
# LVGL 8.3 通过 main/lv_conf.h 配置；禁用与组件 Kconfig 混配
```

> **约束 C-04**：`sdkconfig`（本地产物）不入库，`sdkconfig.defaults` 必须入库；任何构建依赖项（组件版本、分区、Flash/PSRAM）改动都要同步 `dependencies.lock` 与 defaults。

### 8.3 发布

- 每次发版提供**合并烧录镜像**（`esptool.py merge_bin`：bootloader + partition-table + app）。
- 版本号：`config.h` 中 `FW_VERSION` 宏统一维护，配套 CHANGELOG。

---

## 9. 编码约束清单（编码前必须过一遍）

| 编号 | 约束 |
| --- | --- |
| **C-01** | 版本锁定：IDF v6.1.0、LVGL 8.3.x，`dependencies.lock` 入库，禁止升 LVGL v9 |
| **C-02** | 芯片 ID **宽松校验**：IMU=0x6B（LSM6DSR 规格书）/ LIS2MDL=0x40 / 气压计=0x50(BMP388)+0x60(BMP390)；未知值降级+日志，勿硬编码拒绝 |
| **C-03** | SD 引脚映射**已按原理图核实**（CLK=36/CMD=35/D0=37/D1=38/D2=34/D3=33），编码以 `config.h` 宏为准，禁止沿用旧文档矛盾值 |
| **C-04** | `sdkconfig.defaults` + `dependencies.lock` 入库；禁止提交 `sdkconfig` |
| **C-05** | 引脚红线：GPIO26~32、GPIO19/20、GPIO43/44 禁用；strapping 引脚保默认态 |
| **C-06** | 配置收敛：参数进 `config.h`，UI 常量进 `ui_common.h`，字符串进 `strings.h`；禁止魔数 |
| **C-07** | 模块化：前缀 `sensors_ / gnss_ / gpx_ / input_ / ui_ / diagnostics_ / settings_`；内部静态化，头文件只暴露接口 |
| **C-08** | 共享状态只读拷贝快照（`sensors_get_state()`），禁止保存内部指针；共享数据必须锁/队列保护；**ISR 内禁止业务函数** |
| **C-09** | LVGL 只允许 `ui_task` 调用；跨任务 UI 更新走队列/标志，禁止裸调 |
| **C-10** | 一切等待必须带超时（GNSS ACK ≤500 ms）；高优先级任务禁止持锁做低速 I/O |
| **C-11** | 时间一律 UTC；NMEA 年份 80 年窗口；计时用 `esp_timer_get_time()`（μs），禁止 tick 换算 |
| **C-12** | 任务栈（≤8 KB）放内部 RAM；大缓冲（LVGL draw buffer、LCD DMA）放 PSRAM；禁止大数组静态分配 |
| **C-13** | 所有驱动 init 检查 `esp_err_t`；致命错误（显示失败）→ 重启；单传感器失败 → 标记 invalid、UI 显示 `--`、周期重试；GNSS 配置失败 → 降级 1 Hz + 日志，禁止死循环 |
| **C-14** | 诊断格式 `[DIAG][T=xxxxms]`：启动 5 s 自检 1 Hz + 5 s 心跳 + 事件触发；日志输出禁止阻塞业务 |
| **C-15** | 编译开 `-Wall -Werror`；CI 与本地构建产物一致 |
| **C-16** | NVS 只在值变化时写入（防擦写损耗）；key 小写下划线、≤15 字符；GNSS 命令 ACK 成功后再持久化 |
| **C-17** | 固件版本统一维护于 `config.h` `FW_VERSION`；发版必须出合并镜像 + CHANGELOG |

---

## 10. 版本与维护

- **固件体积基线**：约 660.8 KB（0x0A1350），分区剩余约 37%（0x5ECB0）——后续功能需控制增量，保持 ≥30% 余量。
- **配置策略**：P-Box 阈值、按键时长、GNSS 参数等全部集中 `config.h` / 设置菜单，禁止硬编码。
- **数据持久化**：校准数据（IMU/磁力计）写入 NVS `cal` 命名空间，设置写入 `settings_store`，重启自动加载。

欢迎基于该固件拓展更多应用模式或传感器能力（扩展方向见 dev_note §13）。
