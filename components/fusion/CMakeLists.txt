# x-io Technologies Fusion（vendored v1.3.3, MIT）
#
# 上游：https://github.com/xioTechnologies/Fusion
# 本组件不做任何算法改动，仅提供 ESP-IDF 组件包装。
idf_component_register(
    SRCS
        "FusionAhrs.c"
        "FusionBias.c"
        "FusionCompass.c"
        "FusionConvention.c"
        "FusionRemap.c"
    INCLUDE_DIRS "."
)

# ESP32-S3 有硬件单精度 FPU：用标准 sqrtf 取代库内"魔数快速逆平方根"，
# 避免约 1% 的归一化误差污染航向解算。
target_compile_definitions(${COMPONENT_LIB} PUBLIC FUSION_USE_NORMAL_SQRT)
