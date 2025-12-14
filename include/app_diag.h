#pragma once
#include <stdbool.h>
#include <zephyr/sys/atomic.h>
#include <stdatomic.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file app_diag.h
 * @brief 애플리케이션 진단/홀드 제어 인터페이스
 */

extern atomic_t g_diag_log_on; /**< 진단 로그 on/off 전역 플래그 */

/**
 * @brief 현재 진단 로그가 활성화되었는지 여부
 * @return true면 로그 활성화
 */
bool diag_on(void);

/**
 * @brief 광고/주기 제어용 홀드(일시정지) 상태 설정
 * @param on true면 홀드 활성화
 */
void app_set_hold(bool on);

/**
 * @brief 광고/주기 제어용 홀드(일시정지) 상태 조회
 * @return true면 홀드 중
 */
bool app_is_hold(void);

/**
 * @brief 진단 로그 on/off 전역 스위치 조회
 * @return true면 로그 활성화
 */
bool app_diag_log_is_enabled(void);

/**
 * @brief 진단 로그 on/off 전역 스위치 설정
 * @param en true면 로그 활성화
 */
void app_diag_log_enable(bool en);

#ifdef __cplusplus
}
#endif
/* end of app_diag.h */
