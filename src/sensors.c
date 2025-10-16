
#include "sensors.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <hal/nrf_saadc.h>

LOG_MODULE_DECLARE(app, LOG_LEVEL_INF);

/* I2C controller */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* SAADC */
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_buf;

/* ===== (보정용 상수) 하드웨어 값에 맞게 필요 시 조정 ===== */
#define ADC_RESOLUTION_BITS   12
#define ADC_FS_MV             3600.0f    // ← 3.6V 풀스케일로 변경 (GAIN=1/6)
#define ADC_CHANNEL_ID        1     /* AIN1 */
#define ADC_INPUT_POS         NRF_SAADC_INPUT_AIN1

/* NTC 파라미터 (기본: 10k NTC, B=3435, 25°C 기준) */
#define NTC_R0_OHM            10000.0f
#define NTC_BETA              3435.0f
#define NTC_T0_K              298.15f /* 25°C */

/* 분압 회로: VDD ─ NTC ──┬── (ADC) ──┬─ R_PULLDN ─ GND
 *                         └────────────┘
 * Rt = R_PULLDN * (Vout/(VDD - Vout))  ← (NTC가 위, 풀다운이 아래인 경우)
 *
 * VDD는 배터리/레귤레이터에 따라 변할 수 있으므로,
 * 대략 2.18 V로 가정. 더 정확히 하려면 VDD를 별도로 측정/보정해 사용.
 */
#define VDD_MV                2180.0f
#define R_PULLDOWN_OHM        10000.0f   /* 보드 설계에 맞춰 조정: 9.1k~10k 등 */

/* 채널 셋업 1회만 */
static bool adc_ch_inited;

static int adc_setup_once(void)
{
    if (adc_ch_inited)
        return 0;

    if (!device_is_ready(adc_dev))
        return -ENODEV;

/* === ADC 설정: 3.6V 풀스케일 === */
struct adc_channel_cfg ch = {
    .gain             = ADC_GAIN_1_6,          // ← GAIN 변경 (FS = 0.6 / (1/6) = 3.6V)
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10),
    .channel_id       = ADC_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive   = NRF_SAADC_INPUT_AIN1,
#endif
};

    int rc = adc_channel_setup(adc_dev, &ch);
    if (rc) return rc;

    adc_ch_inited = true;
    return 0;
}

/* VDD_SENSEOR 2.18V NTC 1.18V*/
int read_ntc_ain1_cx100(int16_t *cx100)
{
    if (!cx100) return -EINVAL;

    int rc = adc_setup_once();
    if (rc) return rc;

    struct adc_sequence seq = {
        .channels    = BIT(ADC_CHANNEL_ID),
        .buffer      = &adc_buf,
        .buffer_size = sizeof(adc_buf),
        .resolution  = ADC_RESOLUTION_BITS,
        .oversampling = 0,
        .calibrate = false,
    };

    rc = adc_read(adc_dev, &seq);
    if (rc) return rc;

    const float denom = (float)((1U << ADC_RESOLUTION_BITS) - 1U);
    float vout_mv = (adc_buf < 0 ? 0.0f : (float)adc_buf) * (ADC_FS_MV / denom);

    /* 권장: VDD는 상수 고정 말고 실제 측정값을 쓰거나 최소한 현재 값으로 반영 */
    const float vdd_mv = 2180.0f;    // ← 네가 측정한 2.18V로 일단 고정(후술 자동측정 권장)

    /* 범위 체크 */
    if (vout_mv <= 0.0f || vout_mv >= vdd_mv - 1.0f) {
        return -ERANGE;
    }

    /* === 분압 역산: (NTC 위, 풀다운 아래) ===
       R_ntc = R_pull * (VDD - Vout) / Vout
    */
    float rt_ohm = R_PULLDOWN_OHM * ((vdd_mv - vout_mv) / vout_mv);  // ← 수정 포인트

    /* Beta 방정식 → ℃×100 */
    float t_inv = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA) * logf(rt_ohm / NTC_R0_OHM);
    float t_c   = (1.0f / t_inv) - 273.15f;

    int32_t t_cx100 = (int32_t)lroundf(t_c * 100.0f);
    if (t_cx100 > INT16_MAX) t_cx100 = INT16_MAX;
    if (t_cx100 < INT16_MIN) t_cx100 = INT16_MIN;

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

int sensors_init(void)
{
    if (!device_is_ready(i2c0))
        return -ENODEV;
    if (!device_is_ready(adc_dev))
        return -ENODEV;
    /* 필요 시 여기서 adc_setup_once() 호출해 선셋업 가능 */
    return 0;
}