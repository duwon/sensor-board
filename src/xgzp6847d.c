/**
 * @file xgzp6847d.c
 * @brief CFSensor XGZP6847D(001MPGPN) I2C 압력센서 드라이버 구현
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
 *  데이터시트 기반 상수 / 레지스터 정의
 * --------------------------------------------------------------------*/

/** @brief XGZP6847D 기본 I2C 슬레이브 주소 (데이터시트 기준 0x6D) */
// #define XGZP6847_I2C_ADDR 0x6D
#define XGZP6847_I2C_ADDR 0x58 // 0x6D 동작 안함. 센서 읽으면 0x58로 나옴.

/** @brief 데이터 레지스터 주소 */
#define XGZP6847_REG_DATA_MSBR 0x06 /**< PressureData<23:16> */
#define XGZP6847_REG_DATA_CSBR 0x07 /**< PressureData<15:8>  */
#define XGZP6847_REG_DATA_LSBR 0x08 /**< PressureData<7:0>   */
#define XGZP6847_REG_TEMP_MSBR 0x09 /**< TempData<15:8>      */
#define XGZP6847_REG_TEMP_LSBR 0x0A /**< TempData<7:0>       */
#define XGZP6847_REG_CMD 0x30       /**< MeasurementCommandRegister */

/**
 * @brief MeasurementCommand 레지스터 비트 구성
 *
 * 7:4 Sleep_time    (sleep 모드에서만 사용, 여기서는 0)
 * 3   Sco           (Start of conversion, 1 → 변환 시작, 완료 후 자동 0)
 * 2:0 Measurement_control
 *
 * 데이터시트:
 *  - 010b : "temperature conversion immediately followed by a pressure-signal
 *           conversion" (combined conversion)
 *  - 011b : sleep mode conversion
 *
 * “원샷 combined conversion”만 사용 → Sleep_time=0000, Sco=1, MC=010b, CMD 값 = 0b0001_0010 = 0x12
 */
#define XGZP6847_CMD_COMBINED_ONESHOT 0x12

/** @brief 24bit 압력 AD 값에서 sign bit */
#define XGZP6847_PRESSURE_SIGN_BIT (1U << 23)
#define XGZP6847_PRESSURE_FULL_SCALE (1U << 24)

/**
 * @brief XGZP6847D001MPGPN의 K 값
 *
 * - 압력 범위: -100 ~ 1000 kPa
 * - 양압 FS = 1000 kPa
 * - 데이터시트 K-테이블: 500 < P ≤ 1000 kPa → K = 8
 *
 * Pressure(Pa) = Pressure_ADC / K    (양압)
 * Pressure(Pa) = (Pressure_ADC - 2^24) / K  (음압)
 */
#define XGZP6847_K_001MPGPN 8.0f

/** @brief 사용 I2C 컨트롤러 */
static const struct device *xgzp6847_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* 압력 오프셋 (단위: Pa). 보정 전에는 0.0f */

/* 내부 헬퍼: 1바이트 레지스터 읽기 */
static int xgzp_read_reg(const struct device *i2c, uint16_t addr, uint8_t reg, uint8_t *val)
{
    // return i2c_write_read(i2c, addr, &reg, 1, val, 1);
    return i2c_reg_read_byte(i2c, addr, reg, val);
}

static float s_pressure_offset_pa = 0.0f;

/**
 * @brief 센서에 combined one-shot 변환 명령 전송
 *
 * @retval 0    성공
 * @retval <0   I2C 에러
 */
static int xgzp6847_start_conversion(void)
{
    uint8_t cmd = XGZP6847_CMD_COMBINED_ONESHOT;
    int ret = i2c_reg_write_byte(xgzp6847_i2c, XGZP6847_I2C_ADDR, XGZP6847_REG_CMD, cmd);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: write CMD(0x30) failed (err %d)", ret);
    }
    return ret;
}

/**
 * @brief 변환 완료까지 대기 (Sco 비트 폴링 또는 타임아웃)
 *
 * - 데이터시트: Sco=0 이면 변환 완료.
 *
 * 여기서는 최대 5회, 1 ms 간격 폴링 (총 ~10 ms) 후,
 * 여전히 Sco=1 이면 타임아웃으로 간주.
 *
 * @retval 0   성공 (변환 완료)
 * @retval -EIO 타임아웃 또는 읽기 에러
 */
static int xgzp6847_wait_conversion_done(void)
{
    int ret;
    uint8_t status = 0;

    for (int i = 0; i < 10; ++i)
    {
        k_msleep(1);

        ret = i2c_reg_read_byte(xgzp6847_i2c, XGZP6847_I2C_ADDR, XGZP6847_REG_CMD, &status);
        if (ret < 0)
        {
            LOG_ERR("XGZP6847D: read CMD(0x30) failed (err %d)", ret);
            return ret;
        }

        /* Sco bit = bit3, 0이면 변환 완료 */
        if ((status & BIT(3)) == 0U)
        {
            return 0;
        }
    }

    LOG_ERR("XGZP6847D: conversion timeout (CMD=0x%02X)", status);
    return -EIO;
}

/**
 * @brief 압력/온도 원시 AD 값 읽기
 *
 * @param[out] pressure_adc  24bit signed pressure ADC (sign-extended 32bit)
 * @param[out] temperature_adc 16bit signed temperature ADC
 *
 * @retval 0    성공
 * @retval <0   에러
 */
static int xgzp6847_read_raw(int32_t *pressure_adc, int16_t *temperature_adc)
{
    if ((pressure_adc == NULL) || (temperature_adc == NULL))
    {
        return -EINVAL;
    }

    if (!device_is_ready(xgzp6847_i2c))
    {
        LOG_ERR("XGZP6847D: I2C device not ready");
        return -ENODEV;
    }

    int ret = xgzp6847_start_conversion();
    if (ret < 0)
    {
        return ret;
    }

    ret = xgzp6847_wait_conversion_done();
    if (ret < 0)
    {
        return ret;
    }

    uint8_t temp_test = 0;
    ret = xgzp_read_reg(xgzp6847_i2c, XGZP6847_I2C_ADDR, 0x00U, &temp_test);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read T[7:0] failed (err %d)", ret);
        return ret;
    }
    else
    {
        LOG_INF("XGZP6847D: read T[7:0]=0x%02X", temp_test);
    }


    uint8_t buf[5] = {0};

    // 압력 Raw 데이터 읽기 (0x06~0x08)
    ret = xgzp_read_reg(xgzp6847_i2c, XGZP6847_I2C_ADDR, 0x06U, &buf[0]);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read P[23:16] failed (err %d)", ret);
        return ret;
    }
    ret = xgzp_read_reg(xgzp6847_i2c, XGZP6847_I2C_ADDR, 0x07U, &buf[1]);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read P[15:8] failed (err %d)", ret);
        return ret;
    }
    ret = xgzp_read_reg(xgzp6847_i2c, XGZP6847_I2C_ADDR, 0x08U, &buf[2]);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read P[7:0] failed (err %d)", ret);
        return ret;
    }

    ret = xgzp_read_reg(xgzp6847_i2c, XGZP6847_I2C_ADDR, 0x09U, &buf[3]);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read T[15:8] failed (err %d)", ret);
        return ret;
    }
    ret = xgzp_read_reg(xgzp6847_i2c, XGZP6847_I2C_ADDR, 0x0AU, &buf[4]);
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: read T[7:0] failed (err %d)", ret);
        return ret;
    }

    LOG_INF("XGZP6847D: raw data P=0x%02X%02X%02X, T=0x%02X%02X", buf[0], buf[1], buf[2], buf[3], buf[4]);

    /* 압력: 24bit 2's complement → 32bit sign extend */
    uint32_t p_raw_u24 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2]);

    if ((p_raw_u24 & XGZP6847_PRESSURE_SIGN_BIT) != 0U)
    {
        /* 음수: 상위 비트 1로 채워 sign-extend */
        *pressure_adc = (int32_t)(p_raw_u24 | 0xFF000000U);
    }
    else
    {
        *pressure_adc = (int32_t)p_raw_u24;
    }

    /* 온도: 16bit 2's complement */
    uint16_t t_raw_u16 = ((uint16_t)buf[3] << 8) | ((uint16_t)buf[4]);
    *temperature_adc = (int16_t)t_raw_u16;

    return 0;
}

int xgzp6847_read_measurement(xgzp6847_range_t range_type, float *pressure_pa, float *temperature_c)
{
    /* 1) K 값 선택 (kPa 기준) */
    float k = 0.0f;

    switch (range_type)
    {
    case XGZP6847_RANGE_001MPGPN: /* -1 ~ 10 bar → +1000 kPa */
        k = 8.0f;                 /* 500 < P ≤ 1000 → K = 8 */
        break;
    default:
        return -EINVAL;
    }

    if ((pressure_pa == NULL) && (temperature_c == NULL))
    {
        return -EINVAL;
    }

    int32_t p_adc = 0;
    int16_t t_adc = 0;
    int ret = xgzp6847_read_raw(&p_adc, &t_adc);
    if (ret < 0)
    {
        return ret;
    }

    /* 압력 변환 (Pa) */
    if (pressure_pa != NULL)
    {
        /* 데이터시트: Pressure(Pa) = Pressure_ADC / K */
        *pressure_pa = ((float)p_adc) / k;
    }

    /* 온도 변환 (℃) */
    if (temperature_c != NULL)
    {
        /* Temperature(℃) = N / 256 (N: 16bit 2's complement) */
        *temperature_c = ((float)t_adc) / 256.0f;
    }

    return 0;
}

int read_xgzp6847_filtered(xgzp6847_range_t range_type, float *pressure_pa, float *temperature_c, bool apply_offset)
{
    if (pressure_pa == NULL)
    {
        return -EINVAL;
    }

    float samples[10];
    float last_temp = 0.0f;

    /* 10회 연속 측정 */
    for (int i = 0; i < 10; ++i)
    {
        int ret = xgzp6847_read_measurement(range_type, &samples[i], (temperature_c != NULL) ? &last_temp : NULL);
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
    winsor_mean_10f(samples, pressure_pa);

    /* offset 적용 여부 */
    if (apply_offset)
    {
        *pressure_pa -= s_pressure_offset_pa;
    }

    return 0;
}

/**
 * @brief 현재 필터링된 압력값이 0이 되도록 offset 설정
 */
int set_calibration_xgzp6847(xgzp6847_range_t range_type)
{
    float p_pa = 0.0f;

    /* offset 미적용 상태의 필터링된 압력값을 읽는다 */
    int ret = read_xgzp6847_filtered(range_type, &p_pa, NULL, false); /* offset 미적용 */
    if (ret < 0)
    {
        LOG_ERR("XGZP6847D: calibration read failed (err %d)", ret);
        return ret;
    }

    /* offet 값 저장 */
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
