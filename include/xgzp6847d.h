/**
 * @file xgzp6847d.h
 * @brief CFSensor XGZP6847D(001MPGPN) I2C 압력센서 드라이버 (Zephyr)
 *
 * - 디지털 I2C 인터페이스 / 24bit 압력, 16bit 온도
 * - 압력 범위: -100 ~ 1000 kPa (=-1 ~ 10 bar, 게이지)
 * - 반환 단위: 압력 Pa, 온도 ℃
 */

#pragma once

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief XGZP6847D 센서 모델/범위 타입
     *
     * 현재 프로젝트에서는 XGZP6847DC001MPGPN 한 가지만 사용하지만 확장을 위해 enum 형태로 정의.
     */
    typedef enum
    {
        /** XGZP6847DC001MPGPN : -100 ~ 1000 kPa */
        XGZP6847_RANGE_001MPGPN = 0,
    } xgzp6847_range_t;

    /**
     * @brief XGZP6847D 압력/온도 1회 측정 (필터 없음)
     *
     * - I2C 원샷 측정을 수행하고, 24bit 압력 AD + 16bit 온도 AD 를 읽어 데이터시트의 변환식을 이용해 변환
     *
     * @param range_type   센서 압력 범위 타입 (현재는 XGZP6847_RANGE_001MPGPN 만 사용)
     * @param[out] pressure_pa   측정된 압력 값 (단위: Pa, NULL이면 무시)
     * @param[out] temperature_c 측정된 온도 값 (단위: ℃, NULL이면 무시)
     *
     * @retval 0        성공
     * @retval -ENODEV  I2C 디바이스 미준비
     * @retval -EINVAL  파라미터 오류
     * @retval <0       Zephyr I2C 에러 코드
     */
    int xgzp6847_read_measurement(xgzp6847_range_t range_type, float *pressure_pa, float *temperature_c);

    /**
     * @brief XGZP6847D 압력 측정 + 윈저라이즈드 평균 필터 적용
     *
     * - 동일 센서를 10번 연속 측정 후 윈저라이즈드 평균 함수로 노이즈 제거
     * - 온도는 마지막 측정 값 한 번만 사용 (필터 미적용)
     *
     * @param range_type   센서 압력 범위 타입
     * @param[out] pressure_pa   필터 적용된 압력 값 (단위: Pa, NULL 금지)
     * @param[out] temperature_c 마지막 측정 온도 값 (단위: ℃, NULL이면 무시)
     * @param apply_offset 보정 오프셋 적용 여부 (true: 보정값 적용, false: 미적용)
     *
     * @retval 0        성공
     * @retval -ENODEV  I2C 디바이스 미준비
     * @retval -EINVAL  파라미터 오류
     * @retval <0       I2C 통신 에러 (Zephyr errno)
     */
    int read_xgzp6847_filtered(xgzp6847_range_t range_type, float *pressure_pa, float *temperature_c, bool apply_offset);

    /**
     * @brief 현재 측정값이 0 Pa가 되도록 압력 오프셋을 저장
     *
     * @param range_type 센서 압력 범위 타입
     *
     * @retval 0       성공
     * @retval <0      측정 오류 또는 I2C 에러
     */
    int set_calibration_xgzp6847(xgzp6847_range_t range_type);

    /**
     * @brief 보정 offset을 0으로 초기화
     */
    void clear_calibration_xgzp6847(void);

#ifdef __cplusplus
}
#endif
