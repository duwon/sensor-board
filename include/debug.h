#pragma once
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file debug.h
 * @brief 진단/디버그 유틸리티 공개 인터페이스
 */

/**
 * @brief 부팅 직후 1회 실행하는 디버그 루틴 묶음
 */
void debug_run_startup(void);

/**
 * @brief I2C0 버스 주소 스캔(0x03..0x77)
 * @retval 0 성공
 * @retval -ENODEV I2C 디바이스 준비 안됨
 */
int i2c_bus_scan(void);

/**
 * @brief LSM6DSO 장치 WHO_AM_I 확인(0x6A)
 * @retval 0 성공(아이디 일치)
 * @retval 음수값 I2C/디바이스 오류
 */
int lsm6dso_probe(void);

/**
 * @brief 진단 로그 on/off 제어
 * @param en true면 로그 활성화
 */
void app_diag_log_enable(bool en);

/**
 * @brief 진단 로그 활성화 여부 조회
 * @return true면 로그 활성화
 */
bool app_diag_log_is_enabled(void);

#ifdef __cplusplus
}
#endif
/* end of debug.h */
