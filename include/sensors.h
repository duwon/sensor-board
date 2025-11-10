#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>


/** @file sensors.h
 * @brief 센서 인터페이스 정의 파일
 */

/** @brief 센서 공통 초기화 (ADC 채널 준비 등).
 * @see sensors_init
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int sensors_init(void);

/** @brief VDD (mV) 측정: SAADC VDD/4 채널 사용
 * @see read_vdd_mv
 * @param vdd_mv 측정된 VDD 전압 (mV)이 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int read_vdd_mv(int16_t *vdd_mv);

/** @brief NTC 온도 읽기: 섭씨 × 100 단위로 반환 (예: 25.34°C → 2534)
 * @see read_ntc_ain1_cx100
 * @param cx100 계산된 온도 (섭씨 × 100)가 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int read_ntc_ain1_cx100(int16_t *cx100);

int read_ntc(int16_t *temperature);

/** @struct sensor_sample_t
 * @brief 여러 센서 측정값을 통합하여 저장하는 구조체
 */
typedef struct {
    int32_t p_value_x100;     /**< 압력 값 * 100 (단위는 모드에 따라 mmH2O 또는 mbar) */
    int16_t temperature_c_x100; /**< 온도 값 * 100 (NTC 또는 센서 자체 온도) */
    uint8_t battery_pc;       /**< 배터리 잔량 (%) */
    int16_t acc_rms_x100[3];  /**< 가속도 RMS * 100 (선택적) */
    int16_t acc_peak_x100[3]; /**< 가속도 Peak * 100 (선택적) */
} sensor_sample_t;

/** @brief I2C 압력 센서(0x28)에서 데이터를 읽고 변환합니다.
 * @see read_pressure_0x28
 * @param out 압력 값(×100)과 온도 값(×100)이 저장될 sensor_sample_t 구조체 포인터
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int read_pressure_0x28(sensor_sample_t *out);

/** @brief MCU 내부 온도 센서 값을 읽습니다.
 * @see Get_MCU_Temperature
 * @param t_c MCU 온도 (°C)가 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int Get_MCU_Temperature(int8_t *t_c);