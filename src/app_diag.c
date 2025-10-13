#include <zephyr/sys/atomic.h>
#include "app_diag.h"

/* 기본 ON */
static atomic_t g_diag_log_on = ATOMIC_INIT(1);

bool app_diag_log_is_enabled(void)
{
    return atomic_get(&g_diag_log_on);
}

void app_diag_log_enable(bool en)
{
    atomic_set(&g_diag_log_on, en);
}
