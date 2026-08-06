# DOC/SPEC — 芯片与器件规格书存档

本目录存放项目用到的**官方规格书/数据手册（Datasheet）**，供开发时查阅引脚定义、寄存器、电气特性等。

## 目录规划

| 文件 | 器件 | 用途 |
| --- | --- | --- |
| `ESP32-S3_Datasheet_CN.pdf` ✅ | ESP32-S3FH4R2 | 主控芯片规格书（中文）：引脚、GPIO26~32 占用、strapping、电气参数 |
| `LSM6DSR_Datasheet.pdf` ✅ | LSM6DSR | IMU（I²C 0x6A，WHO_AM_I=0x6A） |
| `LIS2MDL_Datasheet.pdf` ✅ | LIS2MDL | 磁力计（I²C 0x1E，WHO_AM_I=0x40） |
| `BMP388_Datasheet.pdf` ✅ | BMP388 | 气压计（I²C 0x76，CHIP_ID=0x50） |
| （待上传） | ST7789 | 显示屏 240×320 |
| （待上传） | MAX-F10S / ATGM336H | GNSS 模块 |
| （待上传） | SD 卡 / 电源 / 充电 IC | 存储与电源管理 |

## 使用约定

- 文件名建议：`<器件型号>_<版本>.<格式>`（如 `ESP32-S3_Technical_Reference_v1.7.pdf`）
- 上传后如需查阅关键参数，请告知文件名，我会读取并提取相关内容
- 规格书为只读存档，禁止在 `main/` 代码中引用其路径
