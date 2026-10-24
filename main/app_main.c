/*
 * app_main.c - 入口（阶段 2：传感器 + LCD + LVGL UI）
 * 流程：NVS -> 传感器汇聚层 -> LCD/LVGL -> sensor_task / diagnostics_task
 */
#include "config.h"
#include "sensors.h"
#include "diagnostics.h"
#include "lcd_driver.h"
#include "ui.h"
#include "gnss.h"
#include "input.h"
#include "pbox.h"
#include "settings_store.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

static const char *TAG = "app_main";

static void sensor_task(void *arg)
{
    for (;;) {
        sensors_update();
        vTaskDelay(pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }
    (void)arg;
}

/* 应用任务：输入事件分发 + P-Box 周期更新（50ms） */
static void app_task(void *arg)
{
    (void)arg;
    for (;;) {
        input_event_t ev;
        while (input_get_event(&ev)) {
            switch (ev) {
            case INPUT_EV_MODE_NEXT:
            case INPUT_EV_MODE_PREV: {
                /* 诊断页调试模式：滚轮由调试页消费（NMEA LOG ↔ SAT INFO） */
                if (input_get_mode() == MODE_DIAG && ui_diag_handle_event(ev)) {
                    break;
                }
                int dir = (ev == INPUT_EV_MODE_NEXT) ? 1 : -1;
                app_mode_t cur_mode = input_get_mode();
                if (cur_mode == MODE_SETTINGS) {
                    setting_substate_t sub = ui_settings_get_substate();
                    if (sub == SETTING_STATE_PAGE) {
                        /* 设置页的页面浏览态：波轮旋转切换大页面 */
                        int next_m = ((int)cur_mode + dir + (int)MODE_MAX) % (int)MODE_MAX;
                        input_set_mode((app_mode_t)next_m);
                    } else {
                        /* 设置页的光标态或编辑态：波轮由设置页处理 */
                        ui_settings_handle_enc(dir);
                    }
                } else {
                    /* 主页面或诊断页：波轮旋转切换大页面 */
                    int next_m = ((int)cur_mode + dir + (int)MODE_MAX) % (int)MODE_MAX;
                    input_set_mode((app_mode_t)next_m);
                }
                break;
            }
            case INPUT_EV_KEY_SHORT:
                if (input_get_mode() == MODE_DIAG && ui_diag_handle_event(ev)) {
                    break;
                }
                if (input_get_mode() == MODE_SETTINGS) {
                    /* 设置页：根据子状态处理短按（进入光标、编辑子项、循环切换或校准退出） */
                    ui_settings_handle_short();
                } else if (input_get_mode() == MODE_MAIN) {
                    main_page_t mp = input_get_main_page();
                    if (mp == MAIN_PAGE_PBOX) {
                        pbox_arm();     /* P-Box：复位或就绪 */
                    } else if (mp == MAIN_PAGE_LOGGER) {
                        ui_logger_start(); /* 轨迹记录：开启短按 */
                    } else if (mp == MAIN_PAGE_BIKE) {
                        ui_bike_toggle_pause(); /* 码表：短按暂停/继续 */
                    }
                }
                break;
            case INPUT_EV_KEY_LONG:
                /* 调试页长按：仅退回健康页；健康页长按仍返回主页面 */
                if (input_get_mode() == MODE_DIAG && ui_diag_handle_event(ev)) {
                    break;
                }
                if (input_get_mode() == MODE_MAIN) {
                    main_page_t mp = input_get_main_page();
                    if (mp == MAIN_PAGE_LOGGER) {
                        ui_logger_stop(); /* 轨迹记录：长按停止 */
                    } else if (mp == MAIN_PAGE_BIKE) {
                        ui_bike_reset_trip(); /* 码表：长按清零 */
                    }
                } else if (input_get_mode() == MODE_SETTINGS) {
                    if (ui_settings_handle_long()) {
                        /* 设置页长按：返回 Page 0 (主页) */
                        input_set_mode(MODE_MAIN);
                        ESP_LOGI(TAG, "long press: return to MODE_MAIN from settings");
                    }
                } else {
                    /* 诊断页：长按返回 Page 0 (主页) */
                    input_set_mode(MODE_MAIN);
                    ESP_LOGI(TAG, "long press: return to MODE_MAIN");
                }
                break;
            case INPUT_EV_KEY_DOUBLE:
                if (input_get_mode() == MODE_DIAG) {
                    ui_diag_handle_event(ev);
                    ESP_LOGI(TAG, "double click: toggle GNSS debug view");
                } else {
                    ESP_LOGI(TAG, "double click event received");
                }
                break;
            default:
                break;
            }
        }
        /* P-Box 输入：GNSS 速度 + 真实线性加速度最大有效分量 */
        gnss_data_t g;
        gnss_get_data(&g);
        sensors_state_t st;
        sensors_get_state(&st);
        float ax = st.imu.valid ? (st.imu.lin_mg[0] / 1000.0f) : 0.0f;
        float ay = st.imu.valid ? (st.imu.lin_mg[1] / 1000.0f) : 0.0f;
        float az = st.imu.valid ? (st.imu.lin_mg[2] / 1000.0f) : 0.0f;
        float acc_thrust = fabsf(ax);
        if (fabsf(ay) > acc_thrust) { acc_thrust = fabsf(ay); }
        if (fabsf(az) > acc_thrust) { acc_thrust = fabsf(az); }
        pbox_update(g.valid ? g.speed_kmh : 0.0f, acc_thrust);

        /* 气压计高度单次自动校准：开机后**等待 GNSS 3D 定位**，
         * 满足 VDOP/vAcc/卫星数条件并连续保持 3s 后校准一次；
         * 每次开机最多一次（不是开机立即校准）。 */
        static bool s_alt_calibrated_this_boot = false;
        static uint32_t s_alt_good_since_ms = 0;
        static bool s_alt_armed_logged = false;
        static bool s_alt_cond_logged = false;
        if (!s_alt_calibrated_this_boot) {
            if (!ui_settings_is_alt_auto_calib_enabled()) {
                s_alt_good_since_ms = 0;
                s_alt_armed_logged = false;
                s_alt_cond_logged = false;
            } else {
                if (!s_alt_armed_logged) {
                    s_alt_armed_logged = true;
                    ESP_LOGI(TAG, "ALT CALIB: armed, waiting for 3D fix + VDOP convergence (sats>=%u, VDOP<=%.1f, vAcc<=%.0fm, hold=%ums)",
                             (unsigned)GNSS_ALT_CALIB_MIN_SATS, (double)GNSS_ALT_CALIB_VDOP_MAX,
                             (double)(GNSS_ALT_CALIB_VACC_MAX_MM / 1000.0f),
                             (unsigned)GNSS_ALT_CALIB_HOLD_MS);
                }
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                bool alt_ok = g.valid && g.fix_type >= 3 &&
                              g.sats >= GNSS_ALT_CALIB_MIN_SATS &&
                              g.vdop > 0.0f && g.vdop <= GNSS_ALT_CALIB_VDOP_MAX &&
                              (g.vacc_mm == 0 || g.vacc_mm <= GNSS_ALT_CALIB_VACC_MAX_MM) &&
                              st.baro.valid;
                if (alt_ok) {
                    if (s_alt_good_since_ms == 0) {
                        s_alt_good_since_ms = now_ms ? now_ms : 1;
                        if (!s_alt_cond_logged) {
                            s_alt_cond_logged = true;
                            ESP_LOGI(TAG, "ALT CALIB: conditions met (VDOP=%.2f, vAcc=%.1fm, sats=%u), hold %ums",
                                     (double)g.vdop, (double)(g.vacc_mm / 1000.0f),
                                     (unsigned)g.sats, (unsigned)GNSS_ALT_CALIB_HOLD_MS);
                        }
                    }
                    if (now_ms - s_alt_good_since_ms >= GNSS_ALT_CALIB_HOLD_MS) {
                        sensors_calibrate_altitude(g.alt_m);
                        s_alt_calibrated_this_boot = true;
                        ESP_LOGI(TAG, "ALT AUTO CALIB: baseline=%.1fm (VDOP=%.2f, vAcc=%.1fm, sats=%u, hold=%ums)",
                                 (double)g.alt_m, (double)g.vdop,
                                 (double)(g.vacc_mm / 1000.0f), (unsigned)g.sats,
                                 (unsigned)GNSS_ALT_CALIB_HOLD_MS);
                    }
                } else {
                    s_alt_good_since_ms = 0;
                    s_alt_cond_logged = false;
                }
            }
        }

        /* 地磁偏角（Magnetic Declination）自动补偿：GNSS 获得有效经纬度后自动解算并注入传感器汇聚层 */
        if (g.valid && (fabs(g.lat) > 0.0001 || fabs(g.lon) > 0.0001)) {
            static bool s_decl_logged = false;
            sensors_update_magnetic_declination(g.lat, g.lon);
            if (!s_decl_logged) {
                float dec = sensors_calc_magnetic_declination(g.lat, g.lon);
                ESP_LOGI(TAG, "MAG DECLINATION: Auto-calibrated = %+.2f deg for GPS Pos (%.4f, %.4f)",
                         (double)dec, g.lat, g.lon);
                s_decl_logged = true;
            }
        }

        /* 加速测试 RUNNING 期间 10ms (100Hz) 高频采样，保证 0.01s 步进；平时 20ms */
        pbox_status_t pb_now;
        pbox_get_status(&pb_now);
        vTaskDelay(pdMS_TO_TICKS(pb_now.state == PBOX_RUNNING ? 10 : 20));
    }
}

static esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void app_main(void)
{
    /* 中国时区：系统/RTC 仍存 UTC（GNSS/GPX 标准），TZ 只影响 localtime() 显示；
     * UI 时钟按 UTC+8 显示。 */
    setenv("TZ", "CST-8", 1);
    tzset();

    /* 1. 开机首要动作：立即硬件拉低 LCD 背光引脚并开启下拉，截断复位期间引脚弱上拉/浮空导致的常亮白屏 */
    lcd_backlight_early_off();

    esp_chip_info_t chip;
    esp_chip_info(&chip);

    /* 串口醒目打印固件版本号与平台信息 */
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, "  ESP32-S3 GPS Performance Analyzer & Outdoor Tracker");
    ESP_LOGI(TAG, "  Firmware Version : %s", FW_VERSION_STR);
    ESP_LOGI(TAG, "  Build Timestamp  : %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "  ESP-IDF Version  : %s", esp_get_idf_version());
    ESP_LOGI(TAG, "  Target Hardware  : ESP32-S3FH4R2 (%u Cores, 4MB Flash, 2MB PSRAM)", (unsigned)chip.cores);
    ESP_LOGI(TAG, "  Display Size     : ST7789 %dx%d Portrait", LCD_H_RES, LCD_V_RES);
    ESP_LOGI(TAG, "==========================================================");

    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(settings_store_init());

    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensors_init 失败: %s", esp_err_to_name(ret));
    }

    /* GNSS（阶段 3）：LDO 使能 + UART1 + 解析任务；失败降级
     * GNSS_DISABLE_FOR_UCENTER=1：仅给模块上电并释放 UART 引脚，
     * 串口交给 u-center 独占（排障用），恢复功能改回 0。 */
#if GNSS_DISABLE_FOR_UCENTER
    if (gnss_power_only_init() != ESP_OK) {
        ESP_LOGE(TAG, "GNSS 仅供电初始化失败");
    }
#else
    if (gnss_init() != ESP_OK) {
        ESP_LOGE(TAG, "GNSS 初始化失败，降级 N/A");
    } else {
        gnss_set_rtc_auto_sync(ui_settings_is_rtc_auto_sync_enabled());
    }
#endif

    /* 输入 + P-Box（阶段 4） */
    pbox_init();
    if (input_init() == ESP_OK) {
        xTaskCreate(app_task, "app_task", TASK_STACK_INPUT, NULL, TASK_PRIO_INPUT, NULL);
    }

    /* LCD + LVGL UI：必须最后初始化——lvgl_port_init 创建的 LVGL 任务会立即运行
     * timer 回调访问各模块快照，须先建好所有 mutex（C-13：失败降级串口自检） */
    if (lcd_driver_init() == ESP_OK) {
        ui_init();
    } else {
        ESP_LOGE(TAG, "LCD 初始化失败，仅串口自检");
    }

    xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SENSOR, NULL, TASK_PRIO_SENSOR, NULL);
    xTaskCreate(diagnostics_task, "diag_task", TASK_STACK_DIAGNOSTIC, NULL, TASK_PRIO_DIAGNOSTIC, NULL);
}
