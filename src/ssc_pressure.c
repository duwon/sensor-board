/**
 * @file ssc_pressure.c
 * @brief Honeywell SSC (SSCDJNNxxxx) I2C 압력 센서 드라이버 구현.
 */

#include "ssc_pressure.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "filter_winsor.h"

LOG_MODULE_REGISTER(ssc_pressure, LOG_LEVEL_INF);

/** @brief SSC 센서의 기본 I2C 7비트 주소 (0x28). */
#define SSC_I2C_ADDR 0x28U

/* I2C0 핸들 */
static const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* 각 범위별 보정 오프셋 저장 (단위는 각 범위의 반환 단위와 동일)
 *  - SSCDJNN010BA2A3 : bar 단위
 *  - SSCDJNN100MD2A3 : mmH2O 단위
 *  - SSCDJNN002ND2A3 : mmH2O 단위
 */
static float s_offset_bar_010BA = 0.0f;
static float s_offset_mmh2o_100MD = 0.0f;
static float s_offset_mmh2o_002ND = 0.0f;

static int ssc_decode_raw(const uint8_t buf[4], ssc_raw_data_t *out)
{
    if (out == NULL)
    {
        return -EINVAL;
    }

    /* Status bits: S1 S0 = D7 D6 of first byte */
    uint8_t status_bits = (buf[0] & 0xC0u) >> 6;

    out->status = (ssc_status_t)status_bits;

    /* Bridge data: 14-bit (B13..B0) */
    out->bridge = (uint16_t)(((uint16_t)(buf[0] & 0x3Fu) << 8) | (uint16_t)buf[1]);

    /* Temperature data: 11-bit (T10..T0) */
    uint16_t temp_raw = (uint16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    temp_raw >>= 5; /* keep upper 11 bits, discard 5 LSBs */

    out->temperature = temp_raw;

    /* Status handling */
    switch (out->status)
    {
    case SSC_STATUS_NORMAL:
        return 0;

    case SSC_STATUS_STALE:
        /* 데이터는 유효하지만 이전 측정값 그대로.
         * 상위 레벨에서 재시도할 수 있도록 -EAGAIN 반환 */
        return -EAGAIN;

    case SSC_STATUS_COMMAND:
    case SSC_STATUS_DIAGNOSTIC:
    default:
        /* 심각한 오류로 간주 */
        return -EIO;
    }
}

int ssc_read_raw(const struct device *i2c, uint16_t addr, ssc_raw_data_t *out)
{
    if ((i2c == NULL) || (out == NULL))
    {
        return -EINVAL;
    }

    uint8_t buf[4] = {0};
    int ret = i2c_read(i2c, buf, sizeof(buf), addr);
    if (ret < 0)
    {
        LOG_ERR("ssc_read_raw: i2c_read failed, ret=%d", ret);
        return ret;
    }

    ret = ssc_decode_raw(buf, out);
    if (ret < 0)
    {
        LOG_WRN("ssc_read_raw: status error=%d (status=%d)", ret, out->status);
    }

    return ret;
}

float ssc_bridge_to_pressure(uint16_t bridge, float p_min, float p_max)
{
    const float out_min = (float)SSC_OUTPUT_MIN_COUNTS;
    const float out_max = (float)SSC_OUTPUT_MAX_COUNTS;
    const float span = out_max - out_min;

    /* 보호 차원에서 span == 0 회피 */
    if (span <= 0.0f)
    {
        return p_min;
    }

    float out = (float)bridge;

    /* 센서 범위 밖 값은 클램프 */
    if (out < out_min)
    {
        out = out_min;
    }
    else if (out > out_max)
    {
        out = out_max;
    }

    float p_span = p_max - p_min;
    float pressure = ((out - out_min) * p_span / span) + p_min;

    return pressure;
}

float ssc_temperature_to_c(uint16_t temp_raw)
{
    /* Equation 3: T(°C) = (DigitalTemp / 2047) * 200 - 50 */
    const float denom = 2047.0f;
    float temp_c = ((float)temp_raw * 200.0f / denom) - 50.0f;

    return temp_c;
}

int ssc_read_measurment(float p_min, float p_max, float *pressure_pa, float *temperature_c)
{
    ssc_raw_data_t raw;
    int ret = ssc_read_raw(i2c0_dev, SSC_I2C_ADDR, &raw);
    if (ret < 0)
    {
        return ret; /* -EIO or -EAGAIN 등 */
    }

    if (pressure_pa != NULL)
    {
        *pressure_pa = ssc_bridge_to_pressure(raw.bridge, p_min, p_max);
    }

    if (temperature_c != NULL)
    {
        *temperature_c = ssc_temperature_to_c(raw.temperature);
    }

    return 0;
}

int read_ssc_filtered(ssc_range_t range_type, float *pressure_pa, float *temperature_c, bool apply_offset)
{
    // pressure_pa는 필터링된 결과를 반환해야 하므로 NULL일 수 없음
    if (pressure_pa == NULL)
        return -EINVAL;

    float samples[WINSOR_SAMPLES_TOTAL], last_temp = 0.0f;

    int ret;

    /* 1) WINSOR_SAMPLES_TOTAL 개 샘플 수집 */
    for (uint32_t i = 0; i < WINSOR_SAMPLES_TOTAL; i++)
    {

        switch (range_type)
        {
        case SSCDJNN010BA2A3:
            /* SSCDJNN010BA2A3 : 0 ~ 10 bar */
            ret = ssc_read_measurment(SSC_010BA2A3_P_MIN_BAR, SSC_010BA2A3_P_MAX_BAR, &samples[i], (temperature_c != NULL) ? &last_temp : NULL);
            if (ret == 0)
            {
                printk("010BA2A3: P=%.3f bar, T=%.2f C\n", (double)samples[i], (double)last_temp);
            }
            else
                return ret;
            break;

        case SSCDJNN100MD2A3:
            /* SSCDJNN100MD2A3 : ±1020 mmH2O */
            ret = ssc_read_measurment(SSC_100MD2A3_P_MIN_MMH2O, SSC_100MD2A3_P_MAX_MMH2O, &samples[i], (temperature_c != NULL) ? &last_temp : NULL);
            if (ret == 0)
            {
                printk("100MD2A3: P=%.1f mmH2O, T=%.2f C\n", (double)samples[i], (double)last_temp);
            }
            else
                return ret;
            break;

        case SSCDJNN002ND2A3:
            /* SSCDJNN002ND2A3 : ±50.8 mmH2O */
            ret = ssc_read_measurment(SSC_002ND2A3_P_MIN_MMH2O, SSC_002ND2A3_P_MAX_MMH2O, &samples[i], (temperature_c != NULL) ? &last_temp : NULL);
            if (ret == 0)
            {
                printk("002ND2A3: P=%.3f mmH2O, T=%.2f C\n", (double)samples[i], (double)last_temp);
            }
            else
                return ret;
            break;

        default:
            break;
        }

        k_sleep(K_MSEC(3)); // 센서 안정화를 위한 짧은 지연
    }

    if (temperature_c != NULL)
    {
        *temperature_c = last_temp; // 온도는 필터링 없이 마지막 측정 값 사용
    }

    /* 2) 윈저라이즈드 평균 필터 적용 */
    int ret2 = winsor_mean_10f(samples, pressure_pa);
    if (ret2 < 0)
        return ret2;

    /* 3) 보정 오프셋 적용 */
    if (apply_offset)
    {
        switch (range_type)
        {
        case SSCDJNN010BA2A3: /* bar */
            *pressure_pa -= s_offset_bar_010BA;
            break;
        case SSCDJNN100MD2A3: /* mmH2O */
            *pressure_pa -= s_offset_mmh2o_100MD;
            break;
        case SSCDJNN002ND2A3: /* mmH2O */
            *pressure_pa -= s_offset_mmh2o_002ND;
            break;
        default:
            break;
        }
    }

    return 0;
}

int set_calibration_ssc(ssc_range_t range_type)
{
    float p = 0.0f;
    float t = 0.0f;

    /* 오프셋 미적용 상태에서 필터링된 압력값을 읽고 저장 */
    int ret = read_ssc_filtered(range_type, &p, &t, false);
    if (ret < 0)
        return ret;

    switch (range_type)
    {
    case SSCDJNN010BA2A3: /* bar */
        s_offset_bar_010BA = p;
        LOG_INF("SSC 010BA offset set to %.5f bar", (double)s_offset_bar_010BA);
        break;
    case SSCDJNN100MD2A3: /* mmH2O */
        s_offset_mmh2o_100MD = p;
        LOG_INF("SSC 100MD offset set to %.2f mmH2O", (double)s_offset_mmh2o_100MD);
        break;
    case SSCDJNN002ND2A3: /* mmH2O */
        s_offset_mmh2o_002ND = p;
        LOG_INF("SSC 002ND offset set to %.2f mmH2O", (double)s_offset_mmh2o_002ND);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

void clear_calibration_ssc(void)
{
    s_offset_bar_010BA = 0.0f;
    s_offset_mmh2o_100MD = 0.0f;
    s_offset_mmh2o_002ND = 0.0f;
    LOG_INF("SSC calibration offsets cleared");
}
