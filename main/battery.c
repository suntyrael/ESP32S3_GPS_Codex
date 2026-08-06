#include "battery.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static float s_pct_smooth = -1.0f;      /* 百分比 10s EMA 平滑（-1=未初始化） */

esp_err_t battery_init(void)
{
    /* ADC2 oneshot 单元 */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BAT_ADC_UNIT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &s_adc_handle), TAG, "adc_oneshot_new_unit failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BAT_ADC_ATTEN,
        .bitwidth = BAT_ADC_BITWIDTH,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg),
                        TAG, "adc_oneshot_config_channel failed");

    /* 曲线拟合校准（失败则退化为近似，打警告） */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .chan = BAT_ADC_CHANNEL,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = BAT_ADC_BITWIDTH,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "curve fitting 校准不可用（%s），使用原始近似", esp_err_to_name(ret));
        s_cali_handle = NULL;
    }

    /* CHG_SAT：输入 + 上拉（开漏输出），低=充电中 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_CHG_SAT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(CHG_SAT) failed");

    /* 注：oneshot 模式采样保持时间由硬件固定（adc_ll_set_sample_cycle 仅影响 digital 模式），
     * 高阻分压源会导致读数偏低 ~20%+波动。解法：分压电阻减至 ≤100k 或加运放缓冲（硬件），
     * 或软件两点线性校准（需第二个实测点）。 */
    ESP_LOGI(TAG, "battery ADC2_CH1 init ok");
    return ESP_OK;
}

esp_err_t battery_read(battery_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int raw = 0;
    /* 多次采样平均：ADC 单次抖动 ±3%，平均后 <1%（BAT_AVG_SAMPLES 次） */
    int raw_sum = 0;
    int raw_cnt = 0;
    for (int i = 0; i < BAT_AVG_SAMPLES; i++) {
        int r = 0;
        if (adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &r) == ESP_OK) {
            raw_sum += r;
            raw_cnt++;
        }
    }
    if (raw_cnt == 0) {
        return ESP_ERR_TIMEOUT;
    }
    raw = raw_sum / raw_cnt;
    data->raw_count = raw;
    int mv = 0;
    if (s_cali_handle != NULL && adc_cali_raw_to_voltage(s_cali_handle, raw, &mv) == ESP_OK) {
        /* 校准电压 */
    } else {
        mv = raw * 3300 / 4095;     /* 无校准时近似 */
    }

    bool saturated = (mv >= (int)BAT_SATURATION_MV);
    data->saturated = saturated;
    data->adc_mv = (uint16_t)mv;
    /* 分压比换算：电池电压 = ADC电压 × BAT_DIVIDER_RATIO；饱和时按量程上限截断 */
    data->voltage_v = (float)(saturated ? BAT_SATURATION_MV : mv) / 1000.0f * (float)BAT_DIVIDER_RATIO;

    /* 线性百分比（3.0V=0%，4.2V=100%） */
    float v = (float)mv / 1000.0f * (float)BAT_DIVIDER_RATIO;
    int pct = (int)((v - (float)BAT_VOLT_EMPTY_MV / 1000.0f) /
                    ((float)(BAT_VOLT_FULL_MV - BAT_VOLT_EMPTY_MV) / 1000.0f) * 100.0f);
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    /* 百分比 10s EMA 平滑：充电中电压波动导致百分比跳变（21%~38%），平滑后稳定 */
    if (s_pct_smooth < 0.0f) {
        s_pct_smooth = (float)pct;
    } else {
        s_pct_smooth += BAT_PCT_EMA_ALPHA * ((float)pct - s_pct_smooth);
    }
    data->percent = (uint8_t)(s_pct_smooth + 0.5f);

#if CHG_SAT_ACTIVE_LOW
    data->charging = (gpio_get_level(PIN_CHG_SAT) == 0);
#else
    data->charging = (gpio_get_level(PIN_CHG_SAT) == 1);
#endif
    return ESP_OK;
}
