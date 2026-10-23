# Fusion 传感器融合库（第三方，原样引入）

本目录为 **x-io Technologies Fusion** 库的 vendored（原样引入）副本，未做任何算法改动。

- 上游仓库：https://github.com/xioTechnologies/Fusion
- 固定版本：**v1.3.3**（commit `9325424011892abacc0ce42b8bb1a8ae20264b9b`）
- 许可证：MIT（见 `LICENSE.md`，Copyright (c) 2021 x-io Technologies）
- 算法：Madgwick 博士论文第 7 章 Revised AHRS（互补滤波 + 四元数积分），
  内置**加速度拒绝**、**磁干扰拒绝**、**超量程自动恢复**与**启动收敛**。

## 为什么直接使用上游库

- 成熟、被广泛验证（MIT，长期维护，有 CI 与单元测试）；
- 自带磁干扰/线性加速度拒绝，正好覆盖本项目"手抖、急加速、靠近磁铁"场景；
- 纯 C99、无动态内存、无 OS 依赖，单次更新约百次级浮点运算，适合 ESP32-S3。

## 本地构建适配

`CMakeLists.txt` 中定义了 `FUSION_USE_NORMAL_SQRT`：
ESP32-S3 具备硬件单精度 FPU，使用标准 `sqrtf` 比库内默认的"魔数快速逆平方根"
精度更高（后者单次牛顿迭代约有 1% 误差），可保证航向解算精度。

## 升级方式

```bash
V=1.3.3   # 改成目标 tag
for f in FusionAhrs.c FusionAhrs.h FusionBias.c FusionBias.h FusionCompass.c \
         FusionCompass.h FusionConfig.h FusionConvention.c FusionConvention.h \
         Fusion.h FusionInline.h FusionMath.h FusionModel.h FusionRemap.c FusionRemap.h; do
  curl -sfL "https://raw.githubusercontent.com/xioTechnologies/Fusion/v$V/Fusion/$f" -o "$f"
done
curl -sfL "https://raw.githubusercontent.com/xioTechnologies/Fusion/v$V/LICENSE.md" -o LICENSE.md
```
