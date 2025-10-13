#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 진단 로그 on/off 전역 스위치 */
bool app_diag_log_is_enabled(void);
void app_diag_log_enable(bool en);

#ifdef __cplusplus
}
#endif
/* end of app_diag.h */