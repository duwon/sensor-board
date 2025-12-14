#pragma once
#include <stdint.h>

/**
 * @file sleep_if.h
 * @brief 저전력 Sleep/ Wake 제어 인터페이스 선언
 */

/**
 * @brief 시스템 인터페이스 전원 다운 후 지정 시간 후 복귀하도록 idle sleep 진입
 * @param seconds 대기 시간(초)
 * @retval 0 성공
 */
int Start_Sleep(uint32_t seconds);

/**
 * @brief Sleep 복귀 후 인터페이스(전원/I2C 핀 상태 등) 재기동
 * @retval 0 성공
 */
int Wakeup(void);
