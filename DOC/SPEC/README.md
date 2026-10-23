# DOC/SPEC — 芯片与器件规格书存档

本目录存放项目用到的**官方规格书/数据手册（Datasheet）**，供开发时查阅引脚定义、寄存器、电气特性等。

## 目录规划

| 文件 | 器件 | 用途 |
| --- | --- | --- |
| `ESP32-S3_Datasheet_CN.pdf` ✅ | ESP32-S3FH4R2 | 主控芯片规格书（中文）：引脚、GPIO26~32 占用、strapping、电气参数 |
| `LSM6DSR_Datasheet.pdf` ✅ | LSM6DSR（实际贴装） | IMU（I²C 0x6A/0x6B，WHO_AM_I=**0x6B**） |
| `LSM6DSV_Datasheet.pdf` ✅ | LSM6DSV（原设计） | IMU 参考：LGA-14L 引脚与 LSM6DSR 逐脚一致 |
| `LIS2MDL_Datasheet.pdf` ✅ | LIS2MDL | 磁力计（I²C 固定 0x1E，WHO_AM_I=0x40） |
| `BMP388_Datasheet.pdf` ✅ | BMP388（实际贴装） | 气压计（I²C 0x76，CHIP_ID=0x50；替代 BMP390L） |
| `u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf` ✅ | u-blox 8 / M8 协议规范 R17 | 覆盖 **UBX protocol 15.00~23.01**；NEO-M8N-0-01 使用 PROTVER 15.00，因此 15.00 章节适用。含 UBX-CFG-PRT/GNSS/RATE/MSG/CFG、NMEA PUBX,40/41、NAV-PVT 等 |
| `NEO-M8_DataSheet_UBX-13003366.pdf` ✅ | NEO-M8 数据手册 R10 | **明确列出 NEO-M8N-0-01：ROM 2.01 / Flash FW 2.01**；引脚、内部 Flash、D_SEL、配置保存说明 |
| `NEO-M8_HardwareIntegrationManual_UBX-13003557.pdf` ✅ | NEO-M8 硬件集成手册 R08 | 电源/V_BCKP/天线/RF 设计参考 |
| （待上传） | ST7789 | 显示屏 240×320 |
| （待上传） | SD 卡 / 电源 / 充电 IC | 存储与电源管理 |

> 说明：UBX-13003221 R17 的产品表按 FW3.01 模块列出（NEO-M8N-0-10），但协议规范本身按 **PROTVER** 分章；当前模块 `$GNTXT` 明确为 `MOD NEO-M8N-0 / PROTVER 15.00 / EXT CORE 2.01`，应查阅 15.00 章节。NEO-M8 数据手册 R10 则明确覆盖 `NEO-M8N-0-01 / ROM 2.01 / Flash FW 2.01`。

## GNSS 配置要点（依据 UBX-13003221 R17）

- **PUBX,41**（PDF p136）：`$PUBX,41,portId,inProto,outProto,baudrate,autobauding*cs`
  - `inProto/outProto` 与 `UBX-CFG-PRT` 的位掩码一致：`0x0001=UBX`、`0x0002=NMEA`、`0x0004=RTCM2`、`0x0008=RTCM3`（协议 <20 不支持 RTCM3）。
  - 可用 `0003`（UBX+NMEA）或 `0007`（UBX+NMEA+RTCM2）启用 UBX 输入/输出。
  - 该命令只负责端口协议/波特率，**不能**配置星座、导航率、动态模型。
- **PUBX,40**（PDF p138）：只配置 NMEA 语句输出率（GGA/RMC/GSA/GSV 等），不改变导航解算率。
- **PUBX,00/03/04**：位置/卫星状态/时间，均为输出语句；`PUBX,03` 可作为 GSV 之外的卫星列表备用来源。
- **UBX-CFG-GNSS**（PDF p176）：`flags` 必须同时设置 `enable` 和至少一个 `sigCfgMask` 信号位，常用 `0x00010001`（使能 + L1C/A）；只写 `0x01` 会因“未配置文件信号”被 NAK。
- **UBX-CFG-RATE**（PDF p221）：协议版本 15 的 `navRate` 固定为 1；`measRate` 100ms=10Hz、1000ms=1Hz。
- **UBX-CFG-CFG**（PDF p166）：`saveMask` 的保留位必须为 0，建议 `0x000003FF`（io/msg/inf/nav/rxm/sen/rinv/ant/log/fts），不要用 `0xFFFF`；`deviceMask=0x03` 表示 BBR+Flash。
- **NEO-M8N 内部 Flash**：UBX 配置可永久保存（数据手册 p14），但当前模块 UBX 通道未打开时无法保存。

## 使用约定

- 文件名建议：`<器件型号>_<版本>.<格式>`（如 `ESP32-S3_Technical_Reference_v1.7.pdf`）
- 上传后如需查阅关键参数，请告知文件名，我会读取并提取相关内容
- 规格书为只读存档，禁止在 `main/` 代码中引用其路径
