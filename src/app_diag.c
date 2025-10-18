#include "app_diag.h"

/* 기본 ON */
atomic_t g_diag_log_on = ATOMIC_INIT(0);

bool diag_on(void) { return atomic_get(&g_diag_log_on); }

bool app_diag_log_is_enabled(void)
{
    return atomic_get(&g_diag_log_on);
}

void app_diag_log_enable(bool en)
{
    atomic_set(&g_diag_log_on, en);
}


static atomic_bool s_hold = ATOMIC_VAR_INIT(false);

void app_set_hold(bool on)
{
    atomic_store(&s_hold, on);
}

bool app_is_hold(void)
{
    return atomic_load(&s_hold);
}