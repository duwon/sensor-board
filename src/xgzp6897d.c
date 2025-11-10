/**
 * @file xgzp6897d.c
 * @brief CFSensor XGZP6897D I2C 압력센서 드라이버 구현
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "xgzp6897d.h"
#include "filter_winsor.h"

LOG_MODULE_REGISTER(xgzp6897d, LOG_LEVEL_INF);

#define XGZP_I2C_ADDR 0x58 /**< XGZP6897D 기본 I2C 주소 */

/* 데이터시트 기준: 24bit signed 값 처리용 상수 */
#define XGZP_SIGN_THRESHOLD (1UL << 23) /*  2^23 =  8,388,608 */
#define XGZP_SIGN_OFFSET (1UL << 24)    /*  2^24 = 16,777,216 */
#define XGZP_SCALE_DENOM (1UL << 21)    /*  2^21 =  2,097,152 */

/* 각 파트의 Full Scale Span (PMAX - PMIN), 단위: Pa
 *  - XGZP6897D001KPDPN : -1 ~ 1 kPa  → span = 2000 Pa
 *  - XGZP6897D010KPDPN : -10 ~ 10 kPa → span = 20000 Pa
 *    (데이터시트의 PMIN/PMAX 정의 참조)
 */
#define XGZP_SPAN_001K_PA (2000.0f)
#define XGZP_SPAN_010K_PA (20000.0f)

/* I2C0 핸들 */
static const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* 내부 헬퍼: 1바이트 레지스터 읽기 */
static int xgzp_read_reg(const struct device *i2c, uint16_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_write_read(i2c, addr, &reg, 1, val, 1);
}

/* 내부 헬퍼: 1바이트 레지스터 쓰기 */
static int xgzp_write_reg(const struct device *i2c, uint16_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(i2c, buf, sizeof(buf), addr);
}

/**
 * @brief XGZP6897D 차압/절대 압력 센서에서 압력과 온도 값을 읽어옵니다.
 *
 * 이 함수는 I2C 통신을 통해 센서 측정을 시작하고, Raw 데이터를 읽은 다음,
 * 데이터 시트에 명시된 공식을 사용하여 Pa(파스칼) 단위의 압력과
 * 섭씨(℃) 단위의 온도로 변환합니다.
 *
 * pressure_pa 또는 temperature_c가 NULL이면 해당 측정 및 변환을 건너뜁니다.
 *
 * @param range_type 센서의 측정 범위 (예: XGZP6897_RANGE_001K, XGZP6897_RANGE_010K).
 * @param pressure_pa 압력 값(Pa)을 저장할 float 포인터. NULL이면 압력 측정 건너뜀.
 * @param temperature_c 온도 값(℃)을 저장할 float 포인터. NULL이면 온도 측정 건너뜀.
 * @return 0 성공. 음수 값은 오류 코드 (예: -ENODEV, -EINVAL).
 */
int xgzp6897_read_measurement(xgzp6897_range_t range_type, float *pressure_pa, float *temperature_c)
{
    // 1) I2C 장치 준비 확인
    if ((i2c0_dev == NULL) || !device_is_ready(i2c0_dev))
    {
        LOG_ERR("XGZP6897D: I2C device not ready");
        return -ENODEV;
    }

    // 최소한 하나라도 측정 요청이 들어왔는지 확인
    if (pressure_pa == NULL && temperature_c == NULL)
    {
        LOG_WRN("XGZP6897D: Both pressure_pa and temperature_c are NULL. No action taken.");
        return 0;
    }

    int ret;
    uint8_t p_h, p_m, p_l; // 압력 Raw 데이터 변수
    uint8_t t_h, t_l;      // 온도 Raw 데이터 변수
    uint32_t pressure_ad = 0;
    uint32_t temperature_ad = 0;

    // --- 측정 시작 및 Raw 데이터 읽기는 압력 또는 온도 중 하나라도 요청되면 수행 ---

    /* 2) 측정 시작: Reg 0x01에 0x01 쓰기 */
    ret = xgzp_write_reg(i2c0_dev, XGZP_I2C_ADDR, 0x01, 0x01);
    if (ret < 0)
    {
        LOG_ERR("XGZP6897D: start measurement failed (err %d)", ret);
        return ret;
    }

    /* 3) 변환 완료까지 대기 (Normal mode typical 5 ms) */
    k_msleep(3);

    // --- 압력 계산 (pressure_pa가 NULL이 아닐 때만) ---
    if (pressure_pa != NULL)
    {

        /* 4) 압력/온도 Raw 데이터 읽기 (0x04~0x08) */
        // 센서가 압력/온도 데이터를 하나의 연속된 레지스터 블록에 저장하므로,
        // 전체 블록을 읽는 것이 효율적이지만, 기존 코드 방식대로 유지하고 NULL 체크만 조정합니다.

        // 압력 Raw 데이터 읽기 (0x04~0x06)
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x04, &p_h);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read P[23:16] failed (err %d)", ret);
            return ret;
        }
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x05, &p_m);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read P[15:8] failed (err %d)", ret);
            return ret;
        }
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x06, &p_l);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read P[7:0] failed (err %d)", ret);
            return ret;
        }

        /* 5) 24bit 압력 AD 값 조합 (pressure_ad) */
        pressure_ad = ((uint32_t)p_h << 16) | ((uint32_t)p_m << 8) | ((uint32_t)p_l);

        float span_pa;

        switch (range_type)
        {
        case XGZP6897_RANGE_001K:
            span_pa = XGZP_SPAN_001K_PA; /* -1~1kPa → span 2000Pa */
            break;
        case XGZP6897_RANGE_010K:
            span_pa = XGZP_SPAN_010K_PA; /* -10~10kPa → span 20000Pa */
            break;
        default:
            return -EINVAL; // 유효하지 않은 range_type
        }

        int32_t signed_pressure;
        if (pressure_ad >= XGZP_SIGN_THRESHOLD)
        {
            signed_pressure = (int32_t)(pressure_ad - XGZP_SIGN_OFFSET);
        }
        else
        {
            signed_pressure = (int32_t)pressure_ad;
        }

        *pressure_pa = ((float)signed_pressure / (float)XGZP_SCALE_DENOM) * span_pa;

        /* 필요하면 여기서 mmH2O 등으로 변환해서 사용 가능
         * 예) mmH2O = Pa / 9.80665f;
         */
    }

    // --- 온도 계산 (temperature_c가 NULL이 아닐 때만) ---
    if (temperature_c != NULL)
    {
        // 온도 Raw 데이터 읽기 (0x07~0x08)
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x07, &t_h);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read T[15:8] failed (err %d)", ret);
            return ret;
        }
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x08, &t_l);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read T[7:0] failed (err %d)", ret);
            return ret;
        }

        /* 16bit 온도 AD 값 조합 (temperature_ad) */
        temperature_ad = ((uint32_t)t_h << 8) | ((uint32_t)t_l);

        int32_t inter_t = (int32_t)temperature_ad;
        if (inter_t >= 32768)
        {
            inter_t -= 65536;
        }

        uint8_t byte1, byte2;

        // 보정 계수 읽기 (0x20, 0x21)
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x20, &byte1);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read 0x20 failed (err %d)", ret);
            return ret;
        }
        ret = xgzp_read_reg(i2c0_dev, XGZP_I2C_ADDR, 0x21, &byte2);
        if (ret < 0)
        {
            LOG_ERR("XGZP6897D: read 0x21 failed (err %d)", ret);
            return ret;
        }

        /* 온도 계산
         * 데이터시트 TEMPERATURE CALCULATION 공식 사용
         *
         * RAW_T = 0x07*2^8 + 0x08
         * Inter_T = (RAW_T > 32768) ? RAW_T - 65536 : RAW_T
         * Byte1 = Reg 0x20 (비트[6:0]: 2의 지수, 비트[7]: 부호)
         * Byte2 = Reg 0x21
         * EOFF = ±(2^(Byte1[6:0]))
         * Shift_N = Byte2 / 10
         * Final_T(℃) = (Inter_T - EOFF) / 2^Shift_N + 25
         */

        /* Byte1 -> EOFF 계산 (sign + exponent) */
        int32_t eoff = 0;
        uint8_t exp = (byte1 & 0x7F);
        if (exp < 31)
        { /* overflow 방지용 간단한 가드 */
            eoff = (int32_t)(1UL << exp);
            if (byte1 & 0x80)
            {
                eoff = -eoff;
            }
        }

        int shift_n = byte2 / 10; /* 보통 7 등 정수값 */

        float denom = (float)(1UL << shift_n);
        *temperature_c = ((float)inter_t - (float)eoff) / denom + 25.0f;
    }

    return 0;
}

/**
 * @brief XGZP6897D 센서에서 압력을 여러 번 측정하고 Winsorized 평균 필터를 적용합니다.
 *
 * 이 함수는 WINSOR_SAMPLES_TOTAL만큼 압력 샘플을 수집한 후, Winsorized 평균을 계산하여 최종 압력 값을 결정
 * 온도 값은 필터링 없이 마지막 측정값을 사용
 *
 * @param range_type 센서의 측정 범위 (예: XGZP6897_RANGE_001K).
 * @param pressure_pa 필터링된 최종 압력 값(Pa)을 저장할 float 포인터. (NULL일 수 없음)
 * @param temperature_c 마지막으로 측정된 온도 값(℃)을 저장할 float 포인터. NULL이면 온도 측정 건너뜀.
 * @return 0 성공. 음수 값은 오류 코드 (-EINVAL, 또는 xgzp6897_read_measurement의 오류 코드).
 */
int read_xgzp6897_filtered(xgzp6897_range_t range_type, float *pressure_pa, float *temperature_c)
{
    // pressure_pa는 필터링된 결과를 반환해야 하므로 NULL일 수 없음
    if (pressure_pa == NULL)
        return -EINVAL;

    float samples[WINSOR_SAMPLES_TOTAL];
    float last_temp = 0.0f; // 온도 Raw 데이터를 임시로 저장할 변수

    /* 1) WINSOR_SAMPLES_TOTAL 개 샘플 수집 */
    for (uint32_t i = 0; i < WINSOR_SAMPLES_TOTAL; i++)
    {
        // pressure_pa는 NULL이 아니므로, &samples[i]에 압력 값을 저장합니다.
        // temperature_c가 NULL이 아닌 경우에만, &last_temp에 온도 값을 저장하도록 합니다.
        int ret = xgzp6897_read_measurement(range_type, &samples[i], temperature_c ? &last_temp : NULL);
        if (ret < 0)
        {
            return ret; // 측정 오류 시 리턴
        }
    }

    if (temperature_c != NULL)
    {
        *temperature_c = last_temp; // 온도는 필터링 없이 마지막 측정 값 사용
    }

    /* 2) 윈저라이즈드 평균 필터 적용 */
    return winsor_mean_10f(samples, pressure_pa);
}