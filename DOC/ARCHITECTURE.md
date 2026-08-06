# 驱动框架设计（ARCHITECTURE）

本文档定义 `ESP32S3_GPS_Codex` 固件的**驱动分层架构、模块接口规范与开发流程约定**，是编码的总纲。与 README.md（硬件/产品规格）、dev_note.md（任务/数据契约）配套使用。

**生效版本**：ESP-IDF `release-v6.1`（CI 镜像）+ LVGL 9.5.0 + esp_lvgl_port 2.8.0

---

## 1. 设计目标与原则

| 原则 | 说明 |
| --- | --- |
| 分层清晰 | 总线层 / 驱动层 / 汇聚层 / 业务层，依赖只允许自上而下 |
| 失败隔离 | 单设备故障不影响其他设备与 UI（每通道 `valid` 标志 + 降级） |
| 单一写者 | 硬件状态只由 `sensor_task` 写入，其余任务读快照（dev_note D-02） |
| 组件化 | 能复用 registry 官方组件（esp_lvgl_port 等）不手写；手写的严格按句柄式规范 |
| 可复现 | 版本锁死 + `sdkconfig.defaults` 入库 + CI 编译验证（C-01/C-04） |
| 不做过度设计 | 不引入设备树、热插拔、跨平台 HAL（见 §10） |

---

## 2. 四层架构

```
┌──────────────────────────────────────────────────────────┐
│ 业务层   ui_* | gpx_logger | diagnostics | input_manager   │ ← 只读快照 / 事件队列
├──────────────────────────────────────────────────────────┤
│ 汇聚层   sensors.c（唯一写者）                              │ ← sensors_update() 采样
│          sensors_get_state() 互斥锁 + 按值拷贝             │     sensors_start_calibration()
├──────────────────────────────────────────────────────────┤
│ 驱动层   每设备一个 .c/.h 模块                              │ ← 各自持有总线设备句柄
│   lsm6dsr | lis2mdl | bmp388 | battery | gnss | sd_card   │     chip ID 宽松校验
├──────────────────────────────────────────────────────────┤
│ 总线层   i2c_bus | spi_bus | uart_gnss | sdmmc            │ ← 初始化一次，多模块共享
└──────────────────────────────────────────────────────────┘
```

依赖规则：
- 总线层 ← 驱动层 ← 汇聚层 ← 业务层（严格单向）
- 驱动层**不感知**其他设备；汇聚层不感知总线细节
- 业务层禁止直接调用驱动/总线 API（一律走快照或队列）

---

## 3. 总线层设计

| 总线 | 引脚 | 设备 | IDF 组件 | 备注 |
| --- | --- | --- | --- | --- |
| I2C0 | SCL=39 / SDA=40（1 MHz） | LSM6DSR、LIS2MDL、BMP388 | `esp_driver_i2c` | `i2c_new_master_bus` 一次；设备各自 `i2c_master_bus_add_device` |
| SPI3（HSPI） | SCK=5/MOSI=8/CS=7/DC=6/RST=4/BL=9 | ST7789 | `esp_lcd` + `esp_driver_spi` | 经 `esp_lcd_panel_io_spi`，背光 PWM 走 LEDC |
| UART1 | TX=17 / RX=18 | NEO-M8N（备选 ATGM336H） | `esp_driver_uart` | 波特率探测 [9600→38400→115200] |
| SDMMC | CLK=36/CMD=35/D0=37/D1=38/D2=34/D3=33 | microSD 4-bit | `esp_driver_sdmmc` + `fatfs` | `/GPX/` 目录 |
| GPIO | 编码器 1/3、按键 2、LDO 14、充电 21 | — | `esp_driver_gpio` | 输入事件走队列 |
| ADC2 | 电池 12（ADC2_CH1） | 分压采样 | `esp_adc` | oneshot + curve fitting |

要点：
- **I2C 总线自带互斥**（新驱动 `i2c_master_transmit` 线程安全）→ 多任务访问同一总线自动串行化；代价是**持总线期间禁止长阻塞操作**
- 总线句柄在 `app_main` 创建后通过 init 参数注入驱动模块，不做全局裸指针
- SDMMC 挂载失败只置状态标志（C-08/D-08），不阻塞启动

---

## 4. 驱动层规范（每设备一个模块）

### 4.1 统一接口模式（句柄式，参照 IDF 新驱动风格）

```c
// xxx.h
typedef struct xxx_dev_s *xxx_handle_t;          // 不透明句柄
esp_err_t xxx_init(i2c_master_bus_handle_t bus, xxx_handle_t *out);
esp_err_t xxx_get_data(xxx_handle_t dev, xxx_data_t *out);
```

- 寄存器表、地址、时序**全部 static 私有**在 .c 内，跨模块零共享
- 每个驱动 `init` 完成：**候选地址探测 → 芯片 ID 宽松校验 → 软复位/默认配置 → 输出句柄**

### 4.2 地址探测与芯片 ID 宽松校验（应对替代料 / SA0 未知）

| 设备 | 候选地址 | 候选 ID | 策略 |
| --- | --- | --- | --- |
| LSM6DSR（原 LSM6DSV 位号） | 0x6A / 0x6B | WHO_AM_I ∈ {0x6B, 0x6A} | `i2c_master_probe` 逐个试，首个 ACK+ID 命中即绑定 |
| LIS2MDL | 0x1E（固定） | WHO_AM_I = 0x40 | 直接绑定，ID 不符打日志降级 |
| BMP388 | 0x76 / 0x77 | CHIP_ID = 0x50 | 同上探测 |
| GNSS | UART 9600 起 | NMEA 校验和 / UBX ACK | 波特率探测 + 协议识别 |

- **ID 校验失败 = 通道降级**（`valid=false`），**绝不**硬编码单一 ID 拒绝启动（C-02）

### 4.3 错误处理约定

- 所有函数返回 `esp_err_t`，调用方必须处理（C-14）
- 运行时读失败：返回错误码，汇聚层标记该通道 `valid=false`，UI 显示 `--`，下一周期自动重试
- 驱动内**禁止打印大段日志**；状态变更用 `ESP_LOGD/ESP_LOGI`，诊断汇总由汇聚层/诊断模块统一输出

---

## 5. 汇聚层（sensors.c）

- `sensors_init()`：依次初始化 I2C 总线 → 各传感器驱动 → ADC/充电 GPIO；读 NVS 校准参数
- `sensors_update()`：**唯一写者**（sensor_task 周期调用）——采样 IMU/磁力计/气压/电量 + 合并 GNSS（`gnss_poll()` 填充），组装 `sensors_state_t`
- `sensors_get_state()`：互斥锁 + **按值拷贝**返回快照，调用方禁止保存内部指针（D-02）
- `sensors_start_calibration()`：后台任务复用驱动接口采样（IMU 512 样本 / 磁力计 600 样本），进度经 `sensors_calibration_status_t` 暴露，NVS `cal` 持久化（D-11）
- 采样节奏（与 GNSS 刷新率解耦）：
  - GNSS：1/5/10/25 Hz（可配），决定 GPX 样本节奏
  - IMU/气压/电量：按需轮询（10~20 Hz）
  - UI：状态栏 1 Hz、仪表 5~10 Hz（D-06）

---

## 6. 业务层

| 模块 | 数据来源 | 说明 |
| --- | --- | --- |
| `ui_*` | `sensors_get_state()` 快照 + 输入队列 | LVGL 9，全部 UI 调用锁在 ui_task（D-05） |
| `gpx_logger` | GPX 样本队列（sensor_task 推送） | 唯一写 SD 的任务（D-08） |
| `diagnostics` | 只读快照 | 启动 5 s 自检 + 5 s 心跳 + 事件触发（C-14） |
| `input_manager` | GPIO 中断 → 事件队列 | 消抖/滤波/多级按键，ISR 只置标志（D-04） |
| `settings_store` | NVS `settings_store` 命名空间 | ACK 成功才落盘（D-03） |

---

## 7. 文件结构

```
main/
├── CMakeLists.txt        # REQUIRES 全量显式声明（见 §8.3）
├── idf_component.yml     # lvgl 9.5.0 + esp_lvgl_port 2.8.0
├── config.h              # 引脚/参数唯一来源（禁止散落魔数，D-12）
├── lv_conf.h             # LVGL 9 配置（如组件需）
├── app_main.c            # 入口：NVS→总线→驱动→任务创建
├── i2c_bus.h / .c        # 总线层
├── lsm6dsr.h / .c        # 驱动层
├── lis2mdl.h / .c
├── bmp388.h / .c
├── battery.h / .c        # ADC oneshot + 饱和保护
├── sd_card.h / .c
├── gnss.h / .c           # NMEA/UBX 解析 + 配置 + 时间同步
├── sensors.h / .c        # 汇聚层
├── input_manager.h / .c
├── gpx_logger.h / .c
├── diagnostics.h / .c
├── settings_store.h / .c
└── ui/
    ├── ui_common.h / ui_state_bar.c / ui_bike_computer.c
    ├── ui_gps_logger.c / ui_pbox.c / ui_gnss_info.c / ui_settings.c
```

---

## 8. 版本与 CI 策略

### 8.1 版本锁定（README §2 同步）

| 项 | 版本 | 说明 |
| --- | --- | --- |
| ESP-IDF | `release-v6.1` | docker hub 无 master/v6.1.0 tag；官方最新 release 分支 |
| LVGL | 9.5.0 | API 与 8.x 不兼容，按 v9 风格写（C-01c） |
| esp_lvgl_port | 2.8.0 | 官方集成组件（tick/显示/输入端口） |
| IDF 组件 | `dependencies.lock` 入库 | 禁止手动改锁 |

### 8.2 CI 工作流（.github/workflows/build.yml）

- 触发：push（任意分支）/ PR
- 步骤：checkout → `espressif/esp-idf-ci-action@v1`（`release-v6.1` + `esp32s3`）→ 上传 build 产物（bin/elf/bootloader/partition）
- 构建日志保存在 GitHub Actions 运行记录中，需要归档时在文档 §9 记录

### 8.3 main/CMakeLists.txt REQUIRES（v6 组件拆分后必须显式全量）

```cmake
idf_component_register(SRCS ... INCLUDE_DIRS ... 
  REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_uart esp_driver_ledc
           esp_driver_sdmmc esp_adc esp_lcd esp_timer esp_psram
           nvs_flash fatfs sdmmc lvgl esp_lvgl_port esp_common)
```

---

## 9. 开发日志规范（DEV_LOG）

**独立保存位置：`DOC/DEV_LOG/`**，与代码同仓、随 git 版本化（不 gitignore）。

| 要求 | 内容 |
| --- | --- |
| 命名 | `YYYY-MM-DD.md`（一天一个文件，可追加多次） |
| 必须记录 | 日期/会话；做了什么；**关键决策及理由**；改动文件清单；CI 编译结果（链接）；发现的问题与对策 |
| 模板 | 见 `DOC/DEV_LOG/README.md` |
| 同步 | 与代码同一次 commit 提交，保证日志与代码状态一一对应 |
| CI 日志 | 构建日志在 GitHub Actions 运行记录（workflow 名 + run id），相关结论写入当天 DEV_LOG |

---

## 10. 明确不做（边界）

| 不做 | 原因 |
| --- | --- |
| 设备树（Device Tree） | ESP-IDF 不走 DT 路线 |
| 动态设备热插拔 | 全部器件板上固定 |
| 跨平台 HAL 抽象层 | 单板单芯片项目，过度抽象增加出错面 |
| 手写 LVGL 显示端口 | 已由 esp_lvgl_port 封装 |
| 传感器驱动强制组件化 | registry 有高质量组件才用，否则自写（按 §4 规范） |

---

## 11. 与既有约束的映射

| 架构条款 | 对应约束 |
| --- | --- |
| §4.2 宽松 ID 校验 | README C-02 |
| §5 单一写者/快照拷贝 | dev_note D-02 |
| §4.3 错误降级 | README C-13 |
| §6 输入队列/ISR 纪律 | dev_note D-04 |
| §6 LVGL 单线程 | dev_note D-05 |
| §8.1 版本锁定 | README C-01 / C-01b / C-01c |
| §7 模块化 | dev_note D-12 / README C-07 |
