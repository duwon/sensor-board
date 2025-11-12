/* ntc.c */
#include "ntc.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <hal/nrf_saadc.h>
#include <math.h>
#include <string.h>
#include "filter_winsor.h"

/** @file sensors.c
 * @brief 보드에 연결된 각종 센서(ADC, I2C)를 제어하고 값을 읽는 함수 구현.
 */

LOG_MODULE_REGISTER(ntc, LOG_LEVEL_INF);

/* ====== ADC 공통 ====== */
/** @brief ADC 노드 정의 */
#define ADC_NODE DT_NODELABEL(adc)
/** @brief ADC 디바이스 구조체 포인터 */
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
/** @brief ADC 변환 결과를 저장할 버퍼 */
static int16_t adc_buf;

/** @brief ADC 해상도 (비트) */
#define ADC_RES_BITS 12

/* GAIN 설정에 따른 풀스케일 전압 (V) */
/** @brief NTC 채널 ADC 게인 설정 (GAIN=1/6) */
#define NTC_GAIN ADC_GAIN_1_6 /* FS = 0.6V / (1/6) = 3.6 V */
/** @brief NTC 채널 풀스케일 전압 (mV) */
#define NTC_FS_MV 3600.0f

/** @brief VDD 채널 ADC 게인 설정 (GAIN=1/6) */
#define VDD_GAIN ADC_GAIN_1_6 /* FS = 0.6V / (1/6) = 3.6 V */
/** @brief VDD 채널 풀스케일 전압 (mV) */
#define VDD_FS_MV 3600.0f

/** @brief ADC 기준 전압 설정 (내부 기준 전압) */
#define ADC_REF_SETTING ADC_REF_INTERNAL
/** @brief ADC 샘플링 시간 설정 (10us) */
#define ADC_ACQ_TIME_SETTING ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)

/* --- 채널 ID: 프로젝트 내에서 고정 --- */
/** @brief NTC 측정에 사용되는 ADC 채널 ID (AIN1 / P0.03) */
#define ADC_CH_NTC 1
/** @brief VDD 측정에 사용되는 내부 ADC 채널 ID (VDD/4) */
#define ADC_CH_VDD 7

/* ====== NTC 파라미터 ====== */
/** @brief NTC의 25°C 저항 (Ohm) */
#define NTC_R0_OHM 10000.0f
/** @brief NTC의 베타 계수 */
#define NTC_BETA 3435.0f
/** @brief 기준 온도 25°C (K) */
#define NTC_T0_K 298.15f

/** @brief NTC와 직렬로 연결된 풀다운 저항 값 (Ohm)
 * NTC는 VDD에 연결되고, 풀다운 저항은 GND에 연결되는 구조를 가정합니다.
 */
#define R_PULLDOWN_OHM 10000.0f

/* 초기화 여부 */
/** @brief NTC ADC 채널이 설정되었는지 여부 */
static bool ch_ntc_inited;
/** @brief VDD ADC 채널이 설정되었는지 여부 */
static bool ch_vdd_inited;

/* Temperature offset in °C x100 applied to read_ntc() results */
static int16_t s_ntc_offset_cx100 = 0;

/** @brief ADC Raw 값을 전압(mV)으로 변환합니다.
 *
 * @param raw ADC 변환 결과 Raw 값
 * @param fs_mv 풀스케일 전압 (mV)
 * @return 측정된 전압 (mV)
 */
static inline float adc_raw_to_mv(int16_t raw, float fs_mv)
{
    const float denom = (float)((1U << ADC_RES_BITS) - 1U);
    if (raw < 0)
        raw = 0;
    return ((float)raw) * (fs_mv / denom);
}

/** @brief NTC 측정용 ADC 채널을 한 번만 설정합니다.
 *
 * @retval 0 성공
 * @retval -ENODEV ADC 디바이스가 준비되지 않았을 때
 * @retval 음수값 ADC 채널 설정 실패 시 Zephyr 에러 코드
 */
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

/** @brief VDD 측정용 ADC 채널을 한 번만 설정합니다.
 *
 * @retval 0 성공
 * @retval -ENODEV ADC 디바이스가 준비되지 않았을 때
 * @retval 음수값 ADC 채널 설정 실패 시 Zephyr 에러 코드
 */
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

/** @brief 센서 공통 초기화 (ADC 채널 준비 등).
 *
 * @retval 0 성공
 * @retval -ENOTSUP ADC가 비활성화된 경우
 * @retval 음수값 ADC 채널 설정 실패 시 Zephyr 에러 코드
 */
int Init_ntc(void)
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

/** @brief VDD (mV)를 측정합니다.
 *
 * 내부 SAADC VDD/4 채널을 사용하여 VDD 전압을 측정하고 변환합니다.
 *
 * @param vdd_mv 측정된 VDD 전압 (mV)이 저장될 포인터
 * @retval 0 성공
 * @retval -EINVAL vdd_mv가 NULL일 때
 * @retval 음수값 ADC 설정/읽기 실패 시 Zephyr 에러 코드
 */
static int read_vdd_mv(int16_t *vdd_mv)
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
    /* VDD/4를 측정했으므로 4를 곱해야 하지만, SAADC VDD 입력이 이미 1/4로 스케일링되어 있습니다.
     * 따라서 VDD = Vmeas * (R_p / R_p) = Vmeas 입니다.
     * nRF52 시리즈의 SAADC VDD 입력은 VDD_FS_MV가 실제 VDD를 나타내는 풀스케일 전압입니다.
     */
    float vdd = 1.0f * v_meas_mv;

    *vdd_mv = (int16_t)lroundf(vdd);
    return 0;
}

/** @brief NTC 저항을 읽어 온도를 섭씨 × 100 단위로 반환합니다.
 *
 * NTC 저항은 VDD와 NTC 사이에 풀다운 저항이 연결된 회로(NTC위/풀다운아래)를 가정하고,
 * 베타 방정식을 사용하여 온도를 계산합니다.
 *
 * @param cx100 계산된 온도 (섭씨 × 100)가 저장될 포인터 (예: 25.34°C → 2534)
 * @retval 0 성공
 * @retval -EINVAL cx100이 NULL일 때
 * @retval -ERANGE Vout이 0 또는 VDD에 너무 가까워서 저항 계산이 불가능할 때
 * @retval 음수값 ADC 설정/읽기 실패 시 Zephyr 에러 코드
 */
static int read_ntc_ain1_cx100(int16_t *cx100)
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

    /* NTC위/풀다운아래 구조의 NTC 저항 계산:
     * Vout = VDD * R_pull / (R_ntc + R_pull)
     * R_ntc = R_pull * (VDD - Vout) / Vout
     */
    if (vout_mv <= 0.0f || vout_mv >= (float)vdd_mv - 1.0f)
        return -ERANGE; /* Vout이 0이나 VDD에 가까우면 계산 오류 */

    float r_ntc = R_PULLDOWN_OHM * ((float)vdd_mv - vout_mv) / vout_mv;

    /* Beta 방정식: 1/T = 1/T0 + (1/Beta) * ln(R_ntc / R0) */
    float t_inv = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0_OHM);
    float t_c = (1.0f / t_inv) - 273.15f; /* K to °C */

    int32_t t_cx100 = (int32_t)lroundf(t_c * 100.0f);

    /* 오버플로우 방지 */
    if (t_cx100 > INT16_MAX)
        t_cx100 = INT16_MAX;
    if (t_cx100 < INT16_MIN)
        t_cx100 = INT16_MIN;

    *cx100 = (int16_t)t_cx100;
    return 0;
}

int read_ntc(int16_t *temperature)
{
    float temp_samples[WINSOR_SAMPLES_TOTAL];
    for (uint32_t i = 0; i < WINSOR_SAMPLES_TOTAL; i++)
    {
        uint16_t temp_cx100 = 0;
        read_ntc_ain1_cx100(&temp_cx100);
        temp_samples[i] = ((float)temp_cx100) / 100.0f;
        if (i + 1 < WINSOR_SAMPLES_TOTAL)
            k_msleep(3);
    }

    float temp_filtered;
    winsor_mean_10f(temp_samples, &temp_filtered);

    int32_t t_cx100 = (int32_t)lroundf(temp_filtered * 100.0f);
    t_cx100 -= (int32_t)s_ntc_offset_cx100; /* apply offset so calibrated point becomes 0 */

    if (t_cx100 > INT16_MAX)
        t_cx100 = INT16_MAX;
    if (t_cx100 < INT16_MIN)
        t_cx100 = INT16_MIN;

    *temperature = (int16_t)t_cx100;
    return 0;
}


/** @brief MCU 내부 온도 센서 값을 읽습니다.
 *
 * nRF Temp 센서 드라이버를 사용하여 MCU 다이 온도를 읽고 섭씨 정수 단위로 반환합니다.
 *
 * @param t_c MCU 온도 (°C)가 저장될 포인터
 * @retval 0 성공
 * @retval -1 센서 디바이스가 준비되지 않았을 때
 * @retval 음수값 센서 샘플 가져오기/채널 읽기 실패 시 Zephyr 에러 코드
 */
int Get_MCU_Temperature(int8_t *t_c)
{
    int err = 0;
    struct sensor_value temp_val;

    const struct device *const temp_sensor = DEVICE_DT_GET_ANY(nordic_nrf_temp);

    if (!device_is_ready(temp_sensor))
    {
        LOG_ERR("Internal Temp sensor not ready!\n");
        return -1;
    }

    // 1. 센서 값 가져오기
    err = sensor_sample_fetch(temp_sensor);
    if (err)
    {
        LOG_ERR("Temp sensor fetch failed: %d\n", err);
        return err;
    }

    // 2. 채널(온도) 값 가져오기
    err = sensor_channel_get(temp_sensor, SENSOR_CHAN_DIE_TEMP, &temp_val);
    if (err)
    {
        LOG_ERR("Temp channel get failed: %d\n", err);
        return err;
    }

    *t_c = (int8_t)temp_val.val1;


    return err;
}
