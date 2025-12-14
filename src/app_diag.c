/**
 * @file
 * @brief 진단 로그 스위치와 홀드 상태 구현
 */
#include "app_diag.h"

/** 진단 로그 스위치 초기값 */
atomic_t g_diag_log_on = ATOMIC_INIT(0);

/**
 * @brief 진단 로그 활성 여부 반환
 */
bool diag_on(void) { return atomic_get(&g_diag_log_on); }

/**
 * @brief 진단 로그 스위치 조회
 */
bool app_diag_log_is_enabled(void)
{
    return atomic_get(&g_diag_log_on);
}

/**
 * @brief 진단 로그 스위치 설정
 * @param en true면 활성
 */
void app_diag_log_enable(bool en)
{
    atomic_set(&g_diag_log_on, en);
}


/** 광고/주기 제어용 홀드 상태 */
static atomic_bool s_hold = ATOMIC_VAR_INIT(false);

/**
 * @brief 홀드 상태 설정
 * @param on true면 홀드
 */
void app_set_hold(bool on)
{
    atomic_store(&s_hold, on);
}

/**
 * @brief 홀드 상태 조회
 * @return true면 홀드 중
 */
bool app_is_hold(void)
{
    return atomic_load(&s_hold);
}
