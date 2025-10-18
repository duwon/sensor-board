#pragma once
#include <stdbool.h>
#include <zephyr/sys/atomic.h>
#include <stdatomic.h>
#ifdef __cplusplus
extern "C" {
#endif

extern atomic_t g_diag_log_on;
bool diag_on(void);

/* 광고/주기 제어용 홀드(일시정지) 상태 */
void app_set_hold(bool on);
bool app_is_hold(void);

/* 진단 로그 on/off 전역 스위치 */
bool app_diag_log_is_enabled(void);
void app_diag_log_enable(bool en);

#ifdef __cplusplus
}
#endif
/* end of app_diag.h */