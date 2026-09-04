# ESP32-S3 多功能 GPS 性能分析设备

## 1. 项目概述

基于 **ESP32-S3FH4R2**（4 MB Flash / 2 MB PSRAM，均封装在片内）的自研固件，集**自行车码表、GPS 轨迹记录仪、汽车 P-Box 性能测试盒、GNSS 信息诊断、设置与校准**于一体。系统围绕 FreeRTOS 与 LVGL 9.5 构建，全部硬件驱动、传感器融合、数据存储与诊断日志自研实现。

- **固件版本**：V0.3.0（release 提供合并烧录包）
- **开发环境**：ESP-IDF v6.1.0
- **目标平台**：ESP32-S3FH4R2（仅此一种，禁止交叉编译到其他芯片）

---

## 2. 版本锁定（禁止漂移）

| 组件 | 锁定版本 | 说明 |
| --- | --- | --- |
| 芯片 | ESP32-S3FH4R2 | 4 MB Flash + 2 MB PSRAM（封装内） |
| ESP-IDF | **release-v6.1**（CI 镜像 tag） | Docker hub 无 master/v6.1.0 tag；release-v6.1 为官方最新 release 分支；新版（如 v6.2）发布后跟随更新并同步本文档 |
| LVGL | **9.5.0**（`"^9.5.0"`） | 官方集成组件 **esp_lvgl_port 2.8.0**（显示/tick/输入端口封装）；LVGL 9 API 与 8.x 不兼容，代码必须按 v9 风格写 |
| 组件来源 | components.espressif.com | 通过 `main/idf_component.yml` 声明 |
| 工具链 | IDF v6.1 自带 | `idf.py` ≥ 5.x 命令行 |

> **约束 C-01**：`dependencies.lock` 必须入库；CI 与本地构建使用完全相同的版本组合，禁止手动改锁文件。
> **约束 C-01b（版本跟随策略）**：CI 镜像锁定 `release-v6.1`（官方最新 release 分支）；官方发布新版本（v6.2+）时，需在 `sdkconfig.defaults`/`idf_component.yml`/workflow 三处同步升级，并用 CI 验证通过后才算切换成功。
> **约束 C-01c（LVGL 9 注意）**：LVGL 9 相对 8.x 有大量重命名：`lv_disp_*`→`lv_display_*`、`lv_scr_act()`→`lv_screen_active()`、flush 回调签名改为 `(lv_display_t*, const lv_area_t*, uint8_t*)`、draw buf 用 `lv_display_set_buffers()`。UI 层由 `esp_lvgl_port` 封装底层，业务代码只调 v9 核心 API。

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
| BAT_ADC / CHG_SAT | 12 / 21 | ADC2_CH1；CHG_SAT 开漏**低=充电中**（TP4054） |
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
- **ADC 量程（已确认硬件事实）**：单节锂电满电 4.2 V，1:1 分压下引脚电压 4.2 V 超出 ADC 校准量程（11 dB 衰减 ~3.1 V）→ **固件必须做饱和保护**：校准后读到 3.1 V 以上按“饱和”处理（满电近似 + 日志），分压比以实测标定为准，禁止信任未校准原始值。
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

### 4.0 通用约定（2026-08-06 用户确认）

| 项 | 约定 |
| --- | --- |
| UI 语言 | **英文**（strings.h 字符串全英文） |
| 时间显示 | 界面显示**本地时间 UTC+8**；GPX 文件 `<time>` **保留 UTC**（D-09 不变） |
| USB（GPIO19/20） | **USB-CDC：固件升级 + 串口**；UART0（43/44）仍为调试日志口 |
| IMU 安装方向 | **设备竖置（长边竖直），屏幕正对用户**；P-Box 纵向加速 = IMU X 轴（水平，车前进方向） |
| 轴向修正 | 竖置安装下重力在 Y 轴；启动自检用重力向量实测校准轴向符号/交换（config.h 参数化，不再沿用旧“Z 反向/Y 不变”假设） |
| 电池 | 单节锂电满电 4.2 V；BAT_ADC=12 为 1:1 分压 → 超 ADC 量程，固件必须饱和保护（见 §3.4） |
| 充电检测 | CHG_SAT（GPIO21）为 **TP4054 开漏输出，充电中低电平**（active-low，config.h 可配极性） |

### 4.1 模式总览与切换（V0.3.0 3 大页面体系）

旋转编码器在所有场景下循环切换 3 个大页面：
`[Page 0: 主功能页] ↔ [Page 1: 传感器诊断页] ↔ [Page 2: 系统设置页]`。

**Page 0 的子视图**由 Page 2 设置项中的 `FUNCTION MODE` 唯一决定：
- `MAIN_PAGE_PBOX`（0）：P-GEAR / P-Box 直线加速性能测试（默认）
- `MAIN_PAGE_LOGGER`（1）：TRACK REC GPS 轨迹记录仪
- `MAIN_PAGE_BIKE`（2）：BIKE COMP 自行车多功能码表

| 操作 | 场景与效果 |
| --- | --- |
| 旋转编码器 | - **Page 0 / Page 1 / Page 2(页面浏览态)**：顺序切换 3 个大页面（Page 0 ↔ Page 1 ↔ Page 2）；<br>- **Page 2(设置光标态)**：上下移动聚焦光标（Item 1~9），视口自动随动平滑滚动；<br>- **Page 2(Function 编辑态)**：波轮轮转切换主功能子项（`P-GEAR` ↔ `TRACK REC` ↔ `BIKE COMP`）。 |
| 短按（SHORT） | - **P-Box 模式**：清零复位（RESET），重新进入就绪待机；<br>- **轨迹记录**：开启轨迹记录（防误触：开启短按）；<br>- **码表模式**：暂停 / 继续单次骑行；<br>- **Page 2(页面浏览态)**：短按进入“设置选择光标态”，激活高亮选择框；<br>- **Page 2(设置光标态)**：<br>&nbsp;&nbsp;• 处于 `FUNCTION MODE` 时：短按进入波轮切换编辑态；再次短按确认选择并退出编辑态；<br>&nbsp;&nbsp;• 处于 2~8 项时：短按循环切换预设值（即刻生效）；<br>&nbsp;&nbsp;• 处于 `SENSOR CALIB` 时：短按进入传感器校准流程；<br>- **校准流程中**：校准结束后界面显示 “校准 OK” 与保存图标，短按保存参数并退出。 |
| 长按（LONG，>900ms） | - **轨迹记录**：停止轨迹记录并安全落盘 GPX（防误触：长按停止）；<br>- **码表模式**：单次骑行里程与极速清零（Trip Reset）；<br>- **Page 1 (诊断页) / Page 2 (设置页任意状态)**：长按快速返回 Page 0 (HOME)；<br>- **校准流程中**：若未进行校准或需取消校准，长按随时安全退出回退至设置菜单上级。 |
| 双击（DOUBLE） | 预留应用层扩展 |

### 4.2 自行车码表（MODE_BIKE_COMPUTER）

- 48 px 速度大字显示；海拔 / 累计里程 / 骑行时间分区块。
- 录制状态指示：未记录=绿色圆圈，记录中=红色闪烁方块。

### 4.3 GPS 轨迹记录仪（MODE_GPS_LOGGER）

- 速度（100 px）/ 轨迹图（120 px）/ 距离（40 px）/ 时间（40 px）。
- 长按控制录制；状态栏显示 GPS / SD / 电池。

### 4.4 P-Box 性能测试（MODE_PBOX）

- 48 px Chakra Petch 科技仪表大字计时，0.01s 精确毫秒步进；实时车速、目标区间、状态胶囊。
- **启动条件（直接踩油门即触发）**：
  - 车辆原本处于静止或低速状态（< 3 km/h）；
  - 深踩油门产生向前推力（综合线性加速度 > 0.25G）或 GPS 车速出现持续抬升（≥ 1.5 km/h），无需手动按键，自动触发计时！
  - **防误触假起步保护**：起步后持续 2.5 秒内车速仍 < 2.0 km/h，自动重置回 `READY` 待机，计时清零，绝不原地空跑。
- **G-Force 雷达仪表**：圆形准星盘，水平 X 轴驱动左右横向 G，垂直 Y 轴驱动纵向推力 G，真实峰值 Peak G 实时捕获。
- **4 宫格测试数据**：初始清零显示 `--.-- s`，真实测得 0-60km/h、0-100km/h、1/4 Mile (402m) 时填入并锁定，显示有效坡度（Slope Valid）。
- 测试完成显示 “FINISHED” 及 “VALID RUN”。

### 4.5 GNSS 与传感器诊断界面（MODE_DIAG）

- **LSM6DSR (6-AXIS IMU)**：RAW ACC 原始三轴总加速度、**LIN ACC 线性加速度（重力分离后纯动态推力，橙色高亮）**、GYRO 三轴角速度。
- **LIS2MDL (3-AXIS MAG)**：三轴磁场强度（uT）、电子罗盘航向角与校准状态。
- **BMP388 (BAROMETER)**：实时气压（hPa）、**三传感器（IMU + MAG + BARO）融合平均温度**、气压高度（m）。
- **NEO-M8N (GNSS)**：定位状态与有效星数、WGS84 经纬度、**海拔高度（ALT）与实时地速（SPD）**、HDOP/VDOP 精度因子。
- **布局设计**：四卡片精确纵向分配（间距 4px），无重叠，充分舒展填充 284px 视口高度。

### 4.6 系统设置菜单（MODE_SETTINGS）

系统设置页采用**三层交互状态机**（`页面浏览态` ↔ `设置光标态` ↔ `子项编辑/校准态`），共涵盖 9 大配置条目，支持垂直列表视口滚动（Scrollable List）与光标聚焦随动：

#### 1. 9 大配置条目与动作规范

| 序号 | 设置项 KEY | 选项值范围 / 动作 | 默认值 | 交互方式 | 硬件底层 / 业务联动 |
|---|---|---|---|---|---|
| 1 | `FUNCTION MODE` | `P-GEAR` / `TRACK REC` / `BIKE COMP` | `P-GEAR` | 短按进入波轮轮转编辑，再次短按确认退出 | 实时决定并联动切换 Page 0 显示子视图 |
| 2 | `COLOR THEME` | `SUNLIGHT` / `DARK` | `SUNLIGHT` | 短按直接循环切换 | 全局切换日光冷银灰 / 暗夜纯黑配色 |
| 3 | `GPS RATE` | `10 HZ` / `18 HZ` / `1 HZ` / `5 HZ` | `10 HZ` | 短按直接循环切换 | 下发 `UBX-CFG-RATE` / `PMTK220`，持久化至 NVS |
| 4 | `GNSS MODE` | `GPS+BDS` / `ALL GNSS` / `GPS ONLY` | `GPS+BDS` | 短按直接循环切换 | 下发 `UBX-CFG-GNSS` / `PMTK353` 切换星座 |
| 5 | `BRIGHTNESS` | `100%` / `80%` / `60%` / `40%` | `100%` | 短按直接循环切换 | 实时调控 GPIO9 LEDC 硬件 PWM 占空比 |
| 6 | `AUTO SLEEP` | `3 MIN` / `5 MIN` / `NEVER` / `1 MIN` | `3 MIN` | 短按直接循环切换 | 启闭 FreeRTOS 软件休眠定时器与低功耗模式 |
| 7 | `RTC AUTO SYNC` | `ON` / `OFF` | `ON` | 短按直接循环切换 | **新增**：GNSS 3D 定位后自动将 UTC 时间同步至系统 RTC |
| 8 | `ALT AUTO CALIB` | `ON` / `OFF` | `ON` | 短按直接循环切换 | **新增**：GNSS 定位后当高度误差 < 5m 时自动单次校准气压计基准，开机仅校准一次 |
| 9 | `SENSOR CALIB` | `START >` | - | 短按进入校准流程 | 触发 IMU 静止零偏校准与磁力计空间 8 字校准流程 |

#### 2. 分层交互逻辑与按键时序
- **进入光标选择**：在设置页处于页面浏览态时，**短按波轮**进入光标选择态，激活选中项高亮边框；
- **光标移动与视口滚动**：在光标选择态下，**旋转波轮**在 1~9 项之间上下移动光标，视口平滑滚动，选中的卡片自动居中显现；
- **Function Mode 独立编辑**：当光标处于第 1 项时，**短按波轮**进入编辑子状态，旋转波轮直接轮转主功能选项，再次**短按波轮**确认并退出编辑子状态，Page 0 立即联动更新；
- **常规参数步进切换**：光标处于 2~8 项时，每次**短按波轮**直接循环轮转下一预设值并即刻生效；
- **校准机制与安全取消**：
  - 光标处于第 9 项 `SENSOR CALIB` 时，短按进入独立校准流程；
  - 屏幕展示校准提示与动态采样进度；
  - 校准算法收敛完成后，界面显著显示 **“校准 OK”** 字样与 **“保存退出”** 图标提示，此时**短按波轮**保存校准矩阵落盘 NVS 并安全退出；
  - 若用户未开始校准或校准中途需放弃，**长按波轮（>900ms）**直接中止并安全回退至上级设置列表；
- **全局返回**：在设置页面任意状态下，**长按波轮（>900ms）**随时直接返回 Page 0 (HOME)。

#### 3. 新增自动同步功能规范
- **RTC 自动同步（`RTC AUTO SYNC`）**：
  - 开关置于 `ON` 时，一旦 GNSS 模块解析出首组有效 RMC/ZDA 且处于 3D 定位状态，系统自动调用 `settimeofday()` 同步系统硬件时钟，并在状态栏更新实时时钟，杜绝手动对时烦恼。
- **气压高度单次自校准（`ALT AUTO CALIB`）**：
  - 开关置于 `ON` 时，开机后持续监测 GNSS 质量因子。当 GNSS 完成 3D Fix 且高度几何精度良好（`VDOP ≤ 2.5` 且垂直精度估计值 `< 5.0m`）时，系统自动采集当前 GNSS 高度，以此反向推算当前海平面参考基准气压 $P_0$（QNH）并校正 BMP388 标定偏移。
  - **单次保护原则**：每次上电开机**仅自动校准一次**，校准成功后置起锁存标志位，后续运行中绝不因进入隧道、林区或建筑阴影处的卫星漂移再次修改气压计，保持气压计绝佳的相对高程变化灵敏度。

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

## 7. UI 规格（ST7789 240×320 竖屏）

- **屏幕规格**：ST7789 240×320 竖屏，SPI3 + DMA 双缓冲。
- **状态栏 20 px**：GPS 图标 + 卫星数（有效绿，未定位红）、10Hz 频度徽章、SD 卡状态（正常绿，录制红闪烁）、本地时钟（UTC+8 / 系统运行秒）、电池电压与百分比（4.1V 80%）、TP4054 充电指示（`CHG`）。
- **视口 284 px**：Page 0 主功能页 / Page 1 传感器诊断页 / Page 2 系统设置页。
- **底部导航栏 16 px**：3 个微小椭圆指示点，当前页高亮拉长。
- **Google Fonts 字体族（全未压缩直读位图点阵）**：
  - `font_chakra_petch_48`（48px）：P-Box 毫秒计时大字、码表速度超大字、轨迹总里程大字。
  - `font_chakra_petch_16`（16px）：仪表卡片主要数值、次要时间。
  - `font_oswald_14`（14px）：卡片标题、设置项名称与选项值。
  - `font_oswald_12`（12px）：状态栏辅助说明、小标签。
  - `font_roboto_mono_12`（12px 等宽）：传感器诊断页等宽对齐数据、状态栏时钟。
  - 全字体挂载 `Montserrat_14` 为安全 Fallback，确保任何符号不丢失。
- **双色彩主题**：
  - **Sunlight 户外高对比度日光模式**：淡冷银灰背景（#EDF2F7）、纯白卡片（#FFFFFF）、深灰边框（#64748B）、极深纯黑文字（#000000），烈日直射下光学对比度 > 18:1。
  - **Dark 暗夜低眩光模式**：纯黑背景（#000000）、深蓝黑卡片（#0B0F17）、高饱和荧光青/绿/橙指示，夜间防眩光。
- **并发纪律**：外部业务任务严禁跨线程直接调用 LVGL 控件，UI 渲染完全收敛于 LVGL 内部任务 `ui_timer_cb`（50Hz / 20ms 周期），杜绝死锁与看门狗超时。

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
# LVGL 9 通过组件 Kconfig（sdkconfig.defaults）配置；禁用 main/lv_conf.h 手写混配
```

> **约束 C-04**：`sdkconfig`（本地产物）不入库，`sdkconfig.defaults` 必须入库；任何构建依赖项（组件版本、分区、Flash/PSRAM）改动都要同步 `dependencies.lock` 与 defaults。

### 8.3 发布

- 每次发版提供**合并烧录镜像**（`esptool.py merge_bin`：bootloader + partition-table + app）。
- 版本号：`config.h` 中 `FW_VERSION` 宏统一维护，配套 CHANGELOG。

---

## 9. 编码约束清单（编码前必须过一遍）

| 编号 | 约束 |
| --- | --- |
| **C-01** | 版本锁定：IDF release-v6.1、**LVGL 9.5.0 + esp_lvgl_port 2.8.0**（registry 组件），`dependencies.lock` 入库 |
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
