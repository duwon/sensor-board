/**
 * @file xgzp6897d.h
 * @brief CFSensor XGZP6897D I2C 압력센서 드라이버 (Zephyr)
 */

#pragma once

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief XGZP6897D 압력 범위 타입
     *
     * - XGZP6897D001KPDPN : -1 ~ 1 kPa  (≈ -100 ~ 100 mmH2O)
     * - XGZP6897D010KPDPN : -10 ~ 10 kPa (≈ -100 ~ 100 mbar)
     */
    typedef enum
    {
        XGZP6897_RANGE_001K = 0, /**< XGZP6897D001KPDPN */
        XGZP6897_RANGE_010K = 1, /**< XGZP6897D010KPDPN */
    } xgzp6897_range_t;

    /**
     * @brief XGZP6897D 센서 한 번 측정 (원샷)
     *
     * @param range_type   센서 압력 범위 타입
     * @param[out] pressure_pa   측정된 압력 값 (단위: Pa, NULL이면 무시)
     * @param[out] temperature_c 측정된 온도 값 (단위: ℃, NULL이면 무시)
     *
     * @retval 0         성공
     * @retval -ENODEV   I2C 디바이스 미준비
     * @retval -EINVAL   파라미터 오류
     * @retval <0        I2C 통신 에러 (Zephyr errno)
     */
    int xgzp6897_read_measurement(xgzp6897_range_t range_type, float *pressure_pa, float *temperature_c);

    /**
     * @brief XGZP6897D 센서 Winsorized 평균 필터 적용 측정
     *
     * @param range_type   센서 압력 범위 타입
     * @param[out] pressure_pa   측정된 압력 값 (단위: Pa, NULL이면 무시)
     * @param[out] temperature_c 측정된 온도 값 (단위: ℃, NULL이면 무시)
     *
     * @retval 0         성공
     * @retval -ENODEV   I2C 디바이스 미준비
     * @retval -EINVAL   파라미터 오류
     * @retval <0        I2C 통신 에러 (Zephyr errno)
     */
    int read_xgzp6897_filtered(xgzp6897_range_t range_type, float *pressure_pa, float *temperature_c);

#ifdef __cplusplus
}
#endif
