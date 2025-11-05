// sleep_if.c
#include "sleep_if.h"
#include "gpio_if.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>

LOG_MODULE_REGISTER(sleep_if, LOG_LEVEL_INF);

#define I2C0_NODE DT_NODELABEL(i2c0)

/* devicetree의 i2c0 pinctrl 설정을 가져온다 */
PINCTRL_DT_DEFINE(I2C0_NODE);
static const struct pinctrl_dev_config *i2c0_pcfg = PINCTRL_DT_DEV_CONFIG_GET(I2C0_NODE);

/* 공통 헬퍼: pinctrl state 적용 */
static int i2c0_apply_state(uint8_t state, const char *state_name)
{
    int ret = pinctrl_apply_state(i2c0_pcfg, state);
    if (ret == -ENOENT)
    {
        /* 이 보드에 sleep state가 아예 없는 경우: 무시하고 통과 */
        LOG_WRN("i2c0: no pinctrl '%s' state (ENOENT), ignore", state_name);
        return 0;
    }
    else if (ret < 0)
    {
        LOG_ERR("i2c0: pinctrl %s state failed %d", state_name, ret);
        return ret;
    }

    return 0;
}

/**
 * @brief I2C 라인을 High-Z(입력, NOPULL)로 설정.
 *
 * 센서 전원을 끄기 전에 호출해서 SCL/SDA를 플로팅 상태로
 */
void i2c_bus_set_hi_z(void)
{
    /* 핀을 Hi-Z 쪽으로 변경 */
    // int ret = pinctrl_apply_state(i2c0_pcfg, PINCTRL_STATE_SLEEP);
    // if (ret)
    // {
    //     LOG_ERR("pinctrl sleep failed %d", ret);
    // }

        (void)i2c0_apply_state(PINCTRL_STATE_SLEEP, "sleep");
}

/**
 * @brief I2C 라인을 기본 상태로 되돌림.
 *
 * 센서 전원 켠 후에 호출해서 SCL/SDA를 기본 상태로 복구
 */
void i2c_bus_restore_default(void)
{
    /* I2C 드라이버가 사용하는 "default" 상태로 핀 설정 */
    // int ret = pinctrl_apply_state(i2c0_pcfg, PINCTRL_STATE_DEFAULT);
    // if (ret)
    // {
    //     LOG_ERR("pinctrl default failed %d", ret);
    // }

        (void)i2c0_apply_state(PINCTRL_STATE_DEFAULT, "default");
}

/* x초 후 깨어남 (간단 idle sleep 버전) */
int Start_Sleep(uint32_t seconds)
{
    /* 1) 인터페이스 전원/기능 다운 */
    i2c_bus_set_hi_z(); // I2C 라인 Hi-Z 설정
    board_led_set(false);
    power_sensor(false);
    power_rpu(false);

    /* 2) 현재 모드(레거시/EXT)에 맞춰 광고 정지 */

    LOG_INF("Enter sleep ~%us", seconds);

    /* 3) 슬립 대기: 이 버전은 시스템오프가 아닌 idle 슬립 */
    // k_sleep(K_SECONDS(seconds)); // loop_fn()에서 다시 스케줄링

    /* 4) 복귀는 Wakeup()에서 수행 */
    return 0;
}

int Wakeup(void)
{
    /* 1) 전원 레일 복구 (순서/안정화 지연) */
    power_rpu(true);
    k_sleep(K_MSEC(2)); // RPU 안정화
    power_sensor(true);
    k_sleep(K_MSEC(5));        // 센서 안정화
    i2c_bus_restore_default(); // I2C 라인 복구

    LOG_INF("Woken up, interfaces re-enabled");
    return 0;
}
