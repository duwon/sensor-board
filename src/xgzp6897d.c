/**
 * @file xgzp6897d.c
 * @brief CFSensor XGZP6897D I2C 압력센서 드라이버 구현
 */

#include "xgzp6897d.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

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

int xgzp6897_read_measurement(xgzp6897_range_t range_type, float *pressure_pa, float *temperature_c)
{
    if ((i2c0_dev == NULL) || !device_is_ready(i2c0_dev))
    {
        LOG_ERR("XGZP6897D: I2C device not ready");
        return -ENODEV;
    }

    /* 1) 측정 시작: Reg 0x01에 0x01 쓰기 */
    int ret = xgzp_write_reg(i2c0_dev, XGZP_I2C_ADDR, 0x01, 0x01);
    if (ret < 0)
    {
        LOG_ERR("XGZP6897D: start measurement failed (err %d)", ret);
        return ret;
    }

    /* 2) 변환 완료까지 대기 (Normal mode typical 5 ms, 여유 있게 20 ms)  */
    k_msleep(20);

    /* 3) 압력/온도 Raw 데이터 읽기 (0x04~0x08) */
    uint8_t p_h, p_m, p_l;
    uint8_t t_h, t_l;

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

    /* 4) 24bit 압력 AD 값 조합 (pressure_ad) */
    uint32_t pressure_ad = ((uint32_t)p_h << 16) | ((uint32_t)p_m << 8) | ((uint32_t)p_l);
    /* 16bit 온도 AD 값 조합 (temperature_ad) */
    uint32_t temperature_ad = ((uint32_t)t_h << 8) | ((uint32_t)t_l);

    /* 5) 압력 계산 (단위: Pa)
     *    데이터시트 PRESSURE CALCULATION 공식 사용
     *
     *    Sum = (0x04*2^16 + 0x05*2^8 + 0x06)
     *    if Sum <  8388608: P = Sum / 2^21 * (PMAX - PMIN)
     *    if Sum >= 8388608: P = (Sum - 2^24) / 2^21 * (PMAX - PMIN)
     */
    if (pressure_pa != NULL)
    {
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
            return -EINVAL;
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

    /* 6) 온도 계산
     *    데이터시트 TEMPERATURE CALCULATION 공식 사용
     *
     *    RAW_T = 0x07*2^8 + 0x08
     *    Inter_T = (RAW_T > 32768) ? RAW_T - 65536 : RAW_T
     *    Byte1 = Reg 0x20 (비트[6:0]: 2의 지수, 비트[7]: 부호)
     *    Byte2 = Reg 0x21
     *    EOFF = ±(2^(Byte1[6:0]))
     *    Shift_N = Byte2 / 10
     *    Final_T(℃) = (Inter_T - EOFF) / 2^Shift_N + 25
     */
    if (temperature_c != NULL)
    {
        int32_t inter_t = (int32_t)temperature_ad;
        if (inter_t > 32768)
        {
            inter_t -= 65536;
        }

        uint8_t byte1, byte2;
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
