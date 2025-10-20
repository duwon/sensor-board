#include "sensors.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>

#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

/* I2C controller */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* ====== ADC 공통 ====== */
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_buf;

/* 채널/FS 설정: 두 채널 모두 동일 gain/ref 사용해 스케일 단순화 */
#define ADC_RES_BITS 12

/* GAIN=1/6 → 0.6 / (1/6) = 3.6 V 풀스케일 */
#define NTC_GAIN ADC_GAIN_1_6 /* FS = 3.6V */
#define NTC_FS_MV 3600.0f

#define VDD_GAIN ADC_GAIN_1_6 /* FS = 0.6V (VDD/4 측정에 적합) */
#define VDD_FS_MV 3600.0f

#define ADC_REF_SETTING ADC_REF_INTERNAL
#define ADC_ACQ_TIME_SETTING ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)

/* --- 채널 ID: 프로젝트 내에서 고정 --- */
#define ADC_CH_NTC 1 /* AIN1 (P0.03) */
#define ADC_CH_VDD 7 /* 임의 ID. VDD/4 내부입력 (핀 미사용) */

/* ====== NTC 파라미터 (보드 실장에 맞게 조정) ====== */
#define NTC_R0_OHM 10000.0f /* 25°C 저항 */
#define NTC_BETA 3435.0f
#define NTC_T0_K 298.15f /* 25°C */

#define R_PULLDOWN_OHM 10000.0f /* NTC 위 / 풀다운 아래 구조 */

/* 초기화 여부 */
static bool ch_ntc_inited;
static bool ch_vdd_inited;

/* raw → mV 변환 */
static inline float adc_raw_to_mv(int16_t raw, float fs_mv)
{
    const float denom = (float)((1U << ADC_RES_BITS) - 1U);
    if (raw < 0)
        raw = 0;
    return ((float)raw) * (fs_mv / denom);
}

static int adc_setup_ntc_once(void)
{
    if (!device_is_ready(adc_dev))
        return -ENODEV;
    if (ch_ntc_inited)
        return 0;

    struct adc_channel_cfg ch = {
        .gain = NTC_GAIN,
        .reference = ADC_REF_SETTING,
        .acquisition_time = ADC_ACQ_TIME_SETTING,
        .channel_id = ADC_CH_NTC,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = NRF_SAADC_INPUT_AIN1,
#endif
    };
    int rc = adc_channel_setup(adc_dev, &ch);
    if (!rc)
        ch_ntc_inited = true;
    return rc;
}

static int adc_setup_vdd_once(void)
{
    if (!device_is_ready(adc_dev))
        return -ENODEV;
    if (ch_vdd_inited)
        return 0;

    struct adc_channel_cfg ch = {
        .gain = VDD_GAIN,
        .reference = ADC_REF_SETTING,
        .acquisition_time = ADC_ACQ_TIME_SETTING,
        .channel_id = ADC_CH_VDD,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = NRF_SAADC_INPUT_VDD, /* 내부 VDD/4 */
#endif
    };
    int rc = adc_channel_setup(adc_dev, &ch);
    if (!rc)
        ch_vdd_inited = true;
    return rc;
}

int sensors_init(void)
{
#if !IS_ENABLED(CONFIG_ADC) || !IS_ENABLED(CONFIG_ADC_NRFX_SAADC)
    return -ENOTSUP;
#else
    int rc;
    rc = adc_setup_vdd_once();
    if (rc)
        return rc;
    rc = adc_setup_ntc_once();
    return rc;
#endif
}

int read_vdd_mv(int16_t *vdd_mv)
{
    if (!vdd_mv)
        return -EINVAL;
    int rc = adc_setup_vdd_once();
    if (rc)
        return rc;

    struct adc_sequence seq = {
        .channels = BIT(ADC_CH_VDD),
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
        .resolution = ADC_RES_BITS,
        .oversampling = 0,
        .calibrate = false,
    };

    rc = adc_read(adc_dev, &seq);
    if (rc)
        return rc;

    /* SAADC 입력 = VDD/4,  전압(mV) */
    float v_meas_mv = adc_raw_to_mv(adc_buf, VDD_FS_MV); /* 이제 FS=3600 */
    float vdd = 1.0f * v_meas_mv;                        /* VDD = 4 × (VDD/4) */

    /* VDD 입력 범위 확인 */
    // if (vdd < 1500.0f || vdd > 5000.0f)
    //     return -ERANGE;

    *vdd_mv = (int16_t)lroundf(vdd);
    return 0;
}

int read_ntc_ain1_cx100(int16_t *cx100)
{
    if (!cx100)
        return -EINVAL;

    int16_t vdd_mv = 0;
    int rc = read_vdd_mv(&vdd_mv);
    if (rc)
        return rc;

    rc = adc_setup_ntc_once();
    if (rc)
        return rc;

    struct adc_sequence seq = {
        .channels = BIT(ADC_CH_NTC),
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
        .resolution = ADC_RES_BITS,
        .oversampling = 0,
        .calibrate = false,
    };

    rc = adc_read(adc_dev, &seq);
    if (rc)
        return rc;

    float vout_mv = adc_raw_to_mv(adc_buf, NTC_FS_MV);

    /* R_ntc = Rpull * (VDD - Vout) / Vout (NTC위/풀다운아래) */
    if (vout_mv <= 0.0f || vout_mv >= (float)vdd_mv - 1.0f)
        return -ERANGE;

    float r_ntc = R_PULLDOWN_OHM * ((float)vdd_mv - vout_mv) / vout_mv;
    /* Beta 식 */
    float t_inv = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0_OHM);
    float t_c = (1.0f / t_inv) - 273.15f;

    int32_t t_cx100 = (int32_t)lroundf(t_c * 100.0f);
    if (t_cx100 > INT16_MAX)
        t_cx100 = INT16_MAX;
    if (t_cx100 < INT16_MIN)
        t_cx100 = INT16_MIN;

    *cx100 = (int16_t)t_cx100;
    return 0;
}

/* Pressure sensor 0x28 (SSC) simple read */
int read_pressure_0x28(sensor_sample_t *out)
{
    if (!device_is_ready(i2c0))
        return -ENODEV;
    uint8_t buf[4] = {0};
    int r = i2c_read(i2c0, buf, sizeof(buf), 0x28);
    if (r)
        return r;
    uint16_t bridge = ((buf[0] & 0x3F) << 8) | buf[1];
    const int OUTPUT_MIN = 1638, OUTPUT_MAX = 14745;
    const int P_MIN = -1020, P_MAX = 1020;
    int32_t p = (int32_t)(bridge - OUTPUT_MIN) * (P_MAX - P_MIN);
    p /= (OUTPUT_MAX - OUTPUT_MIN);
    p += P_MIN;
    out->p_value_x100 = p * 100;
    int16_t t_raw = ((buf[2] << 8) | (buf[3] & 0xE0)) >> 5;
    out->temperature_c_x100 = (int16_t)((t_raw * 977) / 10 - 5000); /* *100 */
    return 0;
}