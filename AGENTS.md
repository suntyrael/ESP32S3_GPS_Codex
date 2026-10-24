# AGENTS.md — ESP32-S3 GPS 项目持久上下文

> 本文件由 Pi 在每次会话自动加载。它只放“跨会话必须记住”的硬约束与决策；
> 详细历史/原理/排障记录放 `dev_note.md`，硬件与接口基线放 `README.md`。

## 1. 项目与硬约束

- 固件：ESP32-S3FH4R2（4 MB Flash / 2 MB PSRAM），**仅此芯片**，禁止交叉编译到其他芯片。
- ESP-IDF：`release-v6.1`；LVGL `9.5.0` + `esp_lvgl_port 2.8.0`（LVGL 9 API，禁止按 v8 教程写）。
- GNSS：**NEO-M8N-0-01**（U39，UBX 协议；OTP 仅授权 GPS+GLONASS，BDS 必 NAK）。
- IMU：LSM6DSRTR（WHO_AM_I=0x6B）；磁力计 LIS2MDL（0x40）；气压计 BMP388（CHIP_ID=0x50/0x60）。
- 屏幕：ST7789 240×320 竖屏；背光 GPIO9 PWM。
- 编码约束：零魔数（`config.h` / `ui_common.h`）；LVGL 只在 `ui_task` 调用；GNSS 共享数据由 `s_mutex` 保护；ACK 等待 ≤500 ms + 重试 1 次 + 降级。

## 2. GNSS 当前基线（2026-10-24 起）

- **不保存配置**：不发送 `CFG-CFG(save)`；每次上电模块回 9600 默认，固件重新下发。
- 首次升级若模块开机在 115200（旧固件保存过），先 `CFG-CFG clear(clearMask=0x03DF, deviceMask=0x03)` 一次；clear 可能触发 ioPort 复位，随后用 9600/115200 双档 MON-VER 确认当前波特率。
- 启动：`9600 → MON-VER → CFG-PRT(115200) → CFG-RATE → CFG-MSG → CFG-GNSS → CFG-NAV5`；默认不跑完整 selftest（`GNSS_SELFTEST_FULL=0`）。
- `CFG-PRT`：115200 / 8N1 / `mode=0x000008D0`（bit4 reserved=1，规范值）/ inProto `0x0007` / outProto `0x0003`。
- `CFG-MSG`：NAV-PVT=1、NAV-SAT=1；GGA/RMC/GSA/GSV=`ceil(导航率/2)`（10Hz→5、5Hz→3）。
- **NAV-SAT 协议 15/18**：version 1、12 字节 block；`+6=prRes`（不是 prn）、`+8=flags`、`elev` 为 I1、`svUsed=bit3(0x08)`。
- **PRN 映射**：NAV-SAT 用 `svId`；GLONASS `+64`（1~32→65~96）、QZSS `+192`（1~10→193~202）；其余 svId 即 PRN。
- `used` 权威源：**UBX-NAV-SAT flags bit3 (svUsed=0x08)**；GSA 只提供 PDOP/HDOP/VDOP（保留 GSA 是因为 VDOP 用于高度校准门限）。
- 计数统一：`gnss_data_t.sats` 在 NAV-SAT 新鲜时用逐星 used；`sats_tracked` = 实际列表条目数；SAT INFO 分母 = `info.count`。
- 时间：系统/RTC 存 UTC；`TZ=CST-8`，UI 显示 UTC+8；GNSS 时间用与 TZ 无关的 `utc_epoch()`。
- 高度自动校准：**开机后等待 3D 定位**；3D fix、sats≥5、VDOP≤1.5、vAcc≤15m（可用时）、连续 3s、气压计有效；满足后校准一次，每次开机最多一次。

## 3. 卫星页规则

- 单列 **20 行**、行距 11px、不翻页；调试页 Tab 在 y=0，sat_box 从 y=16 开始。
- 排序：`used 优先 → CN0 降序 → sys/prn`；未用星保留，只排在后面。
- 超过 20 颗按排名截断，底部显示 `G/R | GSV | AGE | 显示数/总数`。
- 显示字段：`SYS PRN ELE AZI CN0 U`（`*`=used，`-`=unused）。

## 4. 会话记忆机制（重要）

Pi 的模型上下文只来自当前会话的 active branch + 自动加载的上下文文件（本文件、README、dev_note 等）。
**新开会话不会自动继承上一个会话的对话**。跨会话保留信息的正确做法：

1. 继续同一会话：`pi --continue`（最近会话）或 `/resume`（选择器）；`/name` 命名。
2. 把长期有效的事实写进 `AGENTS.md`（本文件）或 `dev_note.md` / `README.md`，每次会话自动加载。
3. 每次改动后更新 `dev_note.md` 的“开发记录”并提交 git；commit message 也是可检索的记忆。
4. 需要分叉实验时用 `/fork` / `/clone`，不要直接开新会话。

## 5. 工作流

- 改 GNSS/UI 前先读 `dev_note.md` §8 / §14 和 `README.md` §4.5 / §6.2。
- 修改后同步更新 `dev_note.md` 开发记录；涉及参数基线时同步 README。
- 构建由 GitHub Actions 执行（`.github/workflows/build.yml`，ESP-IDF release-v6.1，esp32s3）；推送 `main` 或任意分支触发。
- 仓库：`https://github.com/suntyrael/ESP32S3_GPS_Codex`。
