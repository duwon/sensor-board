// sleep_if.c (수정본)
#include "sleep_if.h"
#include "gpio_if.h"
#include "ble_adv.h"                 // ★ 추가: ble_cfg_t, Ble_Stop, Init_Ble, ble_get_last_cfg
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sleep_if, LOG_LEVEL_INF);

/* x초 후 깨어남 (간단 idle sleep 버전) */
int Start_Sleep(uint32_t seconds)
{
    /* 1) 인터페이스 전원/기능 다운 */
    board_led_set(false);
    power_sensor(false);
    power_rpu(false);

    /* 2) 현재 모드(레거시/EXT)에 맞춰 광고 정지 */
    

    LOG_INF("Enter sleep ~%us", seconds);

    /* 3) 슬립 대기: 이 버전은 시스템오프가 아닌 idle 슬립 */
    k_sleep(K_SECONDS(seconds));

    /* 4) 복귀는 Wakeup()에서 수행 */
    return 0;
}

int Wakeup(void)
{
    /* 1) 전원 레일 복구 (순서/안정화 지연) */
    power_rpu(true);
    k_sleep(K_MSEC(2));     // RPU 안정화
    power_sensor(true);
    k_sleep(K_MSEC(5));     // 센서 안정화



    LOG_INF("Woken up, interfaces re-enabled");
    return 0;
}
