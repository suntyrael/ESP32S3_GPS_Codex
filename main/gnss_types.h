#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GNSS_DYNAMIC_PEDESTRIAN = 0,
    GNSS_DYNAMIC_AUTOMOTIVE = 1,
    GNSS_DYNAMIC_SEA = 2,
    GNSS_DYNAMIC_AIRBORNE = 3,
} gnss_dynamic_mode_t;

#ifdef __cplusplus
}
#endif

