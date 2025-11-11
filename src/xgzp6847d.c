/**
 * @file xgzp6847d.c
 * @brief CFSensor XGZP6847D(001MPGPN) I2C 압력센서 드라이버 구현 (V2.9 데이터시트 기준)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>

#include "xgzp6847d.h"
#include "filter_winsor.h"

LOG_MODULE_REGISTER(xgzp6847d, LOG_LEVEL_INF);

/* ----------------------------------------------------------------------
 *  데이터시트 V2.9 기반 상수 / 레지스터 정의
 * --------------------------------------------------------------------*/

/** @brief XGZP6847D 기본 I2C 슬레이브 주소 (데이터시트 기준 0x58) */
#define XGZP6847_I2C_ADDR 0x58

/* 레지스터 주소 */
#define XGZP6847_REG_ID 0x00
#define XGZP6847_REG_CHIP_CONTROL 0x01
#define XGZP6847_REG_CFG_OSR 0x02
#define XGZP6847_REG_CFG_MEAS 0x03
#define XGZP6847_REG_P_DATA_MSB 0x04  /* Data out<23:16> */
#define XGZP6847_REG_P_DATA_MID 0x05  /* Data out<15:8>  */
#define XGZP6847_REG_P_DATA_LSB 0x06  /* Data out<7:0>   */
#define XGZP6847_REG_T_DATA_MSB 0x07  /* Temp out<15:8>  */
#define XGZP6847_REG_T_DATA_LSB 0x08  /* Temp out<7:0>   */
#define XGZP6847_REG_TEMP_COEFF1 0x20 /* Byte1 */
#define XGZP6847_REG_TEMP_COEFF2 0x21 /* Byte2 */

/* Pressure calculation용 상수 (2^21, 2^23, 2^24) */
#define XGZP6847_SUM_SIGN_THRESHOLD 8388608.0f /* 2^23 */
#define XGZP6847_SUM_FULL_SCALE 16777216.0f    /* 2^24 */
#define XGZP6847_SUM_DIVISOR 2097152.0f        /* 2^21 */

/* 현재 사용 모델: XGZP6847DC001MPGPN (-100 ~ 1000 kPa, N+P) */
#define XGZP6847_PMIN_PA_001MPGPN (-100000.0f) /* -100 kPa */
#define XGZP6847_PMAX_PA_001MPGPN (1000000.0f) /* 1000 kPa */

/** @brief 사용 I2C 컨트롤러 (보드 오버레이 기준 i2c0 사용 가정) */
static const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/**
 * @brief 압력 보정 오프셋 (Pa)
 *
 * set_calibration_xgzp6847() 호출 시, 해당 시점의 필터링된 압력값을 저장하여
 * 이후 read_xgzp6847_filtered(..., apply_offset=true)에서 0점 보정으로 적용합니다.
 */
static float s_pressure_offset_pa = 0.0f;

/**
 * @brief 내부: Reg0x01에 0x01을 써서 측정을 트리거
 *
 * 데이터시트 IIC 예제 코드:
 *   Write_One_Byte(0x01, 0x01);
 *
 * @retval 0   성공
 * @retval <0  I2C 에러
 */
static int xgzp6847_trigger_measurement(void)
{
    int ret = i2c_reg_write_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_CHIP_CONTROL, 0x01);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: write CHIP_CONTROL(0x01) failed (err %d)", ret);
    }
    return ret;
}

/**
 * @brief 내부: 압력 계산 (데이터시트 PRESSURE CALCULATION 식)
 *
 * Sum = (0x04 * 2^16 + 0x05 * 2^8 + 0x06)
 *
 * if Sum < 2^23:
 *   P = Sum / 2^21 * (PMAX - PMIN) + PMIN
 * else:
 *   P = (Sum - 2^24) / 2^21 * (PMAX - PMIN) + PMIN
 *
 * @param range_type 센서 범위 타입
 * @param sum        24bit ADC Sum
 * @param[out] pressure_pa 압력(Pa)
 */
static int xgzp6847_convert_pressure(xgzp6847_range_t range_type, uint32_t sum, float *pressure_pa)
{
    if (pressure_pa == NULL)
    {
        return -EINVAL;
    }

    float pmin = 0.0f;
    float pmax = 0.0f;

    switch (range_type)
    {
    case XGZP6847_RANGE_001MPGPN:
        /* -100 ~ 1000 kPa → PMIN=-100000Pa, PMAX=1000000Pa */
        pmin = XGZP6847_PMIN_PA_001MPGPN;
        pmax = XGZP6847_PMAX_PA_001MPGPN;
        break;

    default:
        return -EINVAL;
    }

    const float span = pmax - pmin; /* (PMAX - PMIN) */

    float f_sum = (float)sum;
    float p;

    if (f_sum < XGZP6847_SUM_SIGN_THRESHOLD)
    {
        /* 양압 영역 */
        p = (f_sum / XGZP6847_SUM_DIVISOR) * span + pmin;
    }
    else
    {
        /* 음압 영역: Sum - 2^24 */
        p = ((f_sum - XGZP6847_SUM_FULL_SCALE) / XGZP6847_SUM_DIVISOR) * span + pmin;
    }

    *pressure_pa = p;
    return 0;
}

/**
 * @brief 내부: 온도 보정 계수(Byte1, Byte2) 읽기 및 해석
 *
 * - Reg0x20(Byte1): [6:0]=지수, [7]=부호
 *     Byte1 = ±2^(Bits[6:0])
 *
 * - Reg0x21(Byte2):
 *     Shift_N = Byte2 / 10
 *     (데이터시트 예제: Byte2=0x46 → 70/10=7)
 *
 * 이 구현에서는 Shift_N을 정수로 가정하고,
 *   denom = 2^Shift_N
 * 으로 처리합니다.
 *
 * @param[out] eoff      EOFF 값 (Byte1 해석 결과)
 * @param[out] shift_pow Shift_N (정수, 0~15 정도)
 */
static int xgzp6847_read_temp_coeff(int *eoff, uint8_t *shift_pow)
{
    if ((eoff == NULL) || (shift_pow == NULL))
    {
        return -EINVAL;
    }

    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
    int ret;

    ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_TEMP_COEFF1, &byte1);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read TEMP_COEFF1(0x20) failed (err %d)", ret);
        return ret;
    }

    ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_TEMP_COEFF2, &byte2);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read TEMP_COEFF2(0x21) failed (err %d)", ret);
        return ret;
    }

    /* Byte1 해석: bit[6:0]=지수, bit[7]=부호 */
    uint8_t exp = (uint8_t)(byte1 & 0x7F);
    if (exp > 30)
    {
        /* 시프트 UB 방지용 클램프 */
        exp = 30;
    }

    int base = 1 << exp;
    if ((byte1 & 0x80U) != 0U)
    {
        *eoff = -base;
    }
    else
    {
        *eoff = base;
    }

    /* Shift_N = Byte2 / 10 (정수로 가정) */
    uint8_t shift_n = (uint8_t)(byte2 / 10U);
    if (shift_n > 15U)
    {
        shift_n = 15U;
    }

    *shift_pow = shift_n;

    LOG_DBG("XGZP6847D: temp coeff Byte1=0x%02X, Byte2=0x%02X, EOFF=%d, Shift_N=%u", byte1, byte2, *eoff, *shift_pow);

    return 0;
}

/**
 * @brief 내부: 온도 계산 (데이터시트 TEMPERATURE CALCULATION 식)
 *
 * 1) RAW_T = Reg0x07*2^8 + Reg0x08
 *    Inter_T = RAW_T - 65536 (RAW_T > 32768) else RAW_T
 *
 * 2) Byte1/Byte2(0x20/0x21) → EOFF, Shift_N
 *
 * 3) Final_T = (Inter_T - EOFF) / 2^Shift_N + 25
 */
static int xgzp6847_convert_temperature(uint16_t raw_temp, float *temperature_c)
{
    if (temperature_c == NULL)
    {
        return -EINVAL;
    }

    int eoff = 0;
    uint8_t shift_pow = 0;

    int ret = xgzp6847_read_temp_coeff(&eoff, &shift_pow);
    if (ret < 0)
    {
        return ret;
    }

    /* Inter_T 계산 */
    int32_t inter_t;
    if (raw_temp > 32768U)
    {
        inter_t = (int32_t)raw_temp - 65536;
    }
    else
    {
        inter_t = (int32_t)raw_temp;
    }

    /* 2^Shift_N 계산 */
    uint32_t denom = 1U << shift_pow;
    if (denom == 0U)
    {
        denom = 1U;
    }

    float t = ((float)inter_t - (float)eoff) / (float)denom + 25.0f;
    *temperature_c = t;

    return 0;
}

int xgzp6847_read_measurement(xgzp6847_range_t range_type, float *pressure_pa, float *temperature_c)
{
    /* 둘 다 NULL이면 할 일이 없음 */
    if ((pressure_pa == NULL) && (temperature_c == NULL))
    {
        LOG_WRN("XGZP6847D: Both pressure_pa and temperature_c are NULL. No action taken.");
        return 0;
    }

    if (!device_is_ready(i2c0_dev))
    {
        LOG_ERR("XGZP6847D: I2C device not ready");
        return -ENODEV;
    }

    int ret;

    /* 1) 측정 트리거 (0x01 ← 0x01) */
    ret = xgzp6847_trigger_measurement();
    if (ret < 0)
    {
        return ret;
    }

    /* 2) 변환 완료까지 대기 (보수적으로 5ms) */
    k_msleep(3);

    /* 3) 압력 계산: pressure_pa != NULL 일 때만 0x04~0x06 읽기 */
    if (pressure_pa != NULL)
    {
        uint8_t p_buf[3];

        // 압력 Raw 데이터 읽기 (0x04~0x06)
#if 1
        ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_P_DATA_MSB, &p_buf[0]);
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read P[23:16] failed (err %d)", ret);
            return ret;
        }
        ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_P_DATA_MID, &p_buf[1]);
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read P[15:8] failed (err %d)", ret);
            return ret;
        }
        ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_P_DATA_LSB, &p_buf[2]);
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read P[7:0] failed (err %d)", ret);
            return ret;
        }
#else
        ret = i2c_burst_read(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_P_DATA_MSB, p_buf, sizeof(p_buf));
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read P_DATA (0x04~0x06) failed (err %d)", ret);
            return ret;
        }
#endif
        uint32_t sum = ((uint32_t)p_buf[0] << 16) | ((uint32_t)p_buf[1] << 8) | ((uint32_t)p_buf[2]);
        ret = xgzp6847_convert_pressure(range_type, sum, pressure_pa);
        if (ret < 0)
        {
            return ret;
        }
    }

    /* 4) 온도 계산: temperature_c != NULL 일 때만 0x07,0x08 및 0x20,0x21 사용 */
    if (temperature_c != NULL)
    {
        uint8_t t_buf[2] = {
            0,
        };

        /* 온도 RAW_T 읽기 (0x07, 0x08) */
        ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_T_DATA_MSB, &t_buf[0]);
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read T_DATA_MSB(0x07) failed (err %d)", ret);
            return ret;
        }

        ret = i2c_reg_read_byte(i2c0_dev, XGZP6847_I2C_ADDR, XGZP6847_REG_T_DATA_LSB, &t_buf[1]);
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read T_DATA_LSB(0x08) failed (err %d)", ret);
            return ret;
        }

        uint16_t raw_t = ((uint16_t)t_buf[0] << 8) | (uint16_t)t_buf[1];

        ret = xgzp6847_convert_temperature(raw_t, temperature_c);
        if (ret < 0)
        {
            return ret;
        }
    }

    return 0;
}

int read_xgzp6847_filtered(xgzp6847_range_t range_type, float *pressure_pa, float *temperature_c, bool apply_offset)
{
    if (pressure_pa == NULL)
    {
        return -EINVAL;
    }

    float samples[WINSOR_SAMPLES_TOTAL];
    float last_temp = 0.0f;

    /* WINSOR_SAMPLES_TOTAL 회 연속 측정 */
    for (int i = 0; i < WINSOR_SAMPLES_TOTAL; ++i)
    {
        int ret = xgzp6847_read_measurement(
            range_type,
            &samples[i],
            (temperature_c != NULL) ? &last_temp : NULL);
        if (ret < 0)
        {
            return ret;
        }
    }

    if (temperature_c != NULL)
    {
        *temperature_c = last_temp; /* 온도는 필터 없이 마지막 값 사용 */
    }

    /* 윈저라이즈드 평균 필터 적용 */
    int ret = winsor_mean_10f(samples, pressure_pa);
    if (ret < 0)
    {
        return ret;
    }

    /* 보정 offset 적용 */
    if (apply_offset)
    {
        *pressure_pa -= s_pressure_offset_pa;
    }

    return 0;
}

int set_calibration_xgzp6847(xgzp6847_range_t range_type)
{
    float p_pa = 0.0f;

    /* offset 미적용 상태의 필터링된 압력값을 읽는다 */
    int ret = read_xgzp6847_filtered(range_type, &p_pa, NULL, false);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: calibration read failed (err %d)", ret);
        return ret;
    }

    /* 오프셋 값 저장 */
    s_pressure_offset_pa = p_pa;

    LOG_INF("XGZP6847D: calibration offset set to %.3f Pa",
            (double)s_pressure_offset_pa);

    return 0;
}

void clear_calibration_xgzp6847(void)
{
    s_pressure_offset_pa = 0.0f;
    LOG_INF("XGZP6847D: calibration offset cleared");
}
