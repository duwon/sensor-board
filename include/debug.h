#pragma once
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 부팅 직후 한 번 실행하는 디버그 러너 */
void debug_run_startup(void);

/* 개별 함수도 필요 시 직접 호출 가능 */
int i2c_bus_scan(void);
int lsm6dso_probe(void);

/* 로그 on/off 제어 API (쉘에서 사용) */
void app_diag_log_enable(bool en);
bool app_diag_log_is_enabled(void);


#ifdef __cplusplus
}
#endif
/* end of debug.h */