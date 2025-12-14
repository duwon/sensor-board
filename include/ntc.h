#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>


/** @file ntc.h
 * @brief NTC ADC 인터페이스 정의 파일
 */

/** @brief 센서 공통 초기화 (ADC 채널 준비 등).
 * @see Init_ntc
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int Init_ntc(void);

/** @brief NTC 온도 읽기: 섭씨 × 100 단위로 반환 (예: 25.34°C → 2534)
 * @see read_ntc
 * @param cx100 계산된 온도 (섭씨 × 100)가 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int read_ntc(int16_t *temperature);

/** @brief MCU 내부 온도 센서 값을 읽습니다.
 * @see Get_MCU_Temperature
 * @param t_c MCU 온도 (°C)가 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int Get_MCU_Temperature(int8_t *t_c);
