# ESP32S3_GPS_Codex

基础工程使用 ESP-IDF v6.1，按功能模块划分组件并提供 FreeRTOS 任务框架、GNSS/传感器/输入/RTC 等驱动占位。

## 目录结构
- `components/`：GNSS、传感器、输入、UI、GPX、P-GEAR、RTC、日志、电源管理组件。
- `main/`：应用入口与任务/队列/状态机框架。
- `CMakeLists.txt`：顶层工程定义。

## 构建与烧录
1. 安装 ESP-IDF v6.1，并执行 `export.sh`/`export.ps1` 载入工具链。
2. 进入项目根目录，首次可以运行 `idf.py set-target esp32s3`。
3. 编译：`idf.py build`
4. 烧录：`idf.py -p <PORT> flash`
5. 监视日志：`idf.py -p <PORT> monitor`

> UI 使用 LVGL + LovyanGFX 的占位接口，需根据实际硬件与显示驱动补充实现。
