/**
 * @file ssc_pressure.h
 * @brief Honeywell SSC (SSCDJNNxxxx) I2C 압력 센서 드라이버.
 *
 * - 디지털 출력 압력 센서 (Honeywell TruStability SSC)
 * - 전달 함수(Transfer function): 14비트 카운트의 10% ~ 90% 범위 (0x0666 ~ 0x3999)
 * - 지원 모델:
 *   SSCDJNN010BA2A3 : 0 ~ 10 bar
 *   SSCDJNN100MD2A3 : ±1020 mmH2O
 *   SSCDJNN002ND2A3 : ±50.8 mmH2O
 */

#ifndef SSC_PRESSURE_H_
#define SSC_PRESSURE_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Honeywell 디지털 전달 함수의 한계값 (14비트 카운트). */
#define SSC_OUTPUT_MIN_COUNTS 1638U  /**< 2^14의 10% (0x0666). */
#define SSC_OUTPUT_MAX_COUNTS 14745U /**< 2^14의 90% (0x3999). */

    /**
     * @brief SSC 압력 범위 타입
     *
     *      SSCDJNN010BA2A3 : 0 ~ 10 bar
     *      SSCDJNN100MD2A3 : ±1020 mmH2O
     *      SSCDJNN002ND2A3 : ±50.8 mmH2O
     */
    typedef enum
    {
        SSCDJNN010BA2A3 = 0, 
        SSCDJNN100MD2A3 = 1, 
        SSCDJNN002ND2A3 = 2
    } ssc_range_t;

    /** @brief 첫 번째 데이터 바이트에서 디코딩된 상태 비트. */
    typedef enum
    {
        SSC_STATUS_NORMAL = 0,    /**< 00: 정상 작동, 유효한 데이터. */
        SSC_STATUS_COMMAND = 1,   /**< 01: 명령 모드 (일반적인 사용에서는 발생하지 않아야 함). */
        SSC_STATUS_STALE = 2,     /**< 10: 오래된 데이터 (너무 빠르게 폴링함). */
        SSC_STATUS_DIAGNOSTIC = 3 /**< 11: 진단 오류. */
    } ssc_status_t;

    /**
     * @brief SSC 센서로부터 읽은 Raw 데이터 구조체.
     */
    typedef struct
    {
        uint16_t bridge;      /**< 14비트 브리지 (압력) 카운트. */
        uint16_t temperature; /**< 11비트 온도 카운트. */
        ssc_status_t status;  /**< 첫 번째 바이트에서 디코딩된 상태 비트. */
    } ssc_raw_data_t;

/** @name SSCDJNN010BA2A3 : 0 ~ 10 bar
 * @note bar 단위로 쓰고 싶으면 p_min=0, p_max=10 그대로 사용.
 * @{
 */
#define SSC_010BA2A3_P_MIN_BAR 0.0f
#define SSC_010BA2A3_P_MAX_BAR 10.0f
/** @} */

/** @name SSCDJNN100MD2A3 : ±1020 mmH2O
 * @note mmH2O 단위.
 * @{
 */
#define SSC_100MD2A3_P_MIN_MMH2O (-1020.0f)
#define SSC_100MD2A3_P_MAX_MMH2O (1020.0f)
/** @} */

/** @name SSCDJNN002ND2A3 : ±50.8 mmH2O
 * @note mmH2O 단위.
 * @{
 */
#define SSC_002ND2A3_P_MIN_MMH2O (-50.8f)
#define SSC_002ND2A3_P_MAX_MMH2O (50.8f)
    /** @} */

    /**
     * @brief SSC 센서로부터 4바이트 Raw 데이터 패킷을 읽어옵니다.
     *
     * @param i2c   I2C 장치 포인터.
     * @param addr   7비트 I2C 주소 (일반적으로 @ref SSC_I2C_ADDR_0X28).
     * @param[out] out 채울 Raw 데이터 구조체.
     *
     * @retval 0 성공 시.
     * @retval -EIO I2C 오류 또는 진단 오류 발생 시.
     * @retval -EAGAIN 오래된 데이터(stale data)일 때 (상태=SSC_STATUS_STALE).
     */
    int ssc_read_raw(const struct device *i2c, uint16_t addr, ssc_raw_data_t *out);

    /**
     * @brief 브리지 카운트를 센서 범위를 사용하여 압력으로 변환합니다.
     *
     * 전달 함수 (Honeywell Equation 2):
     * Pressure = ( (Output - Outputmin) * (Pmax - Pmin) /
     *        (Outputmax - Outputmin) ) + Pmin
     *
     * @param bridge Raw 14비트 브리지 카운트 (0 ~ 16383).
     * @param p_min  Outputmin에서의 압력 (단위: bar, mmH2O 등).
     * @param p_max  Outputmax에서의 압력 (p_min과 동일 단위).
     *
     * @return p_min/p_max와 동일한 단위의 압력 값.
     */
    float ssc_bridge_to_pressure(uint16_t bridge, float p_min, float p_max);

    /**
     * @brief 11비트 온도 카운트를 섭씨 온도로 변환합니다.
     *
     * 공식 (Honeywell Equation 3):
     * Temperature(°C) = (DigitalTemp / 2047) * 200 - 50
     *
     * @param temp_raw 11비트 Raw 온도 카운트.
     *
     * @return 섭씨 단위의 온도.
     */
    float ssc_temperature_to_c(uint16_t temp_raw);

    /**
     * @brief 상위 레벨 도우미 함수: 압력 및 온도를 읽고 변환합니다.
     *
     * @param i2c   I2C 장치 포인터.
     * @param addr   7비트 I2C 주소 (0x28).
     * @param p_min  범위 최솟값 (단위 별도 정의).
     * @param p_max  범위 최댓값.
     * @param[out] p  계산된 압력 값 (p_min/p_max와 동일 단위).
     * @param[out] t  섭씨 온도 (NULL 이면 무시).
     *
     * @retval 0 성공 시.
     * @retval -EIO I2C 오류 또는 진단 오류 발생 시.
     * @retval -EAGAIN 오래된 데이터(stale data)일 때.
     */
    int ssc_read_pressure(const struct device *i2c, uint16_t addr, float p_min, float p_max, float *p, float *t);

#ifdef __cplusplus
}
#endif

#endif /* SSC_PRESSURE_H_ */