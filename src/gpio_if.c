
#include "gpio_if.h"
#include <zephyr/sys/printk.h>

static struct board_gpio g_cfg = {
    .led        = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), led0_gpios, {0}),
    .btn        = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), btn0_gpios, {0}),
    .io_int     = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), io_int_gpios, {0}),
    .en_sensor  = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), en_sensor_gpios, {0}),
    .en_rpu     = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), en_rpu_gpios, {0}),
    .soh_alarm  = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), soh_alarm_gpios, {0}),
    .soh_ok     = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), soh_ok_gpios, {0}),
};

int board_gpio_init(void)
{
    const struct gpio_dt_spec *pins[] = {
        &g_cfg.led, &g_cfg.btn, &g_cfg.io_int, &g_cfg.en_sensor, &g_cfg.en_rpu, &g_cfg.soh_alarm, &g_cfg.soh_ok,
    };
    gpio_flags_t flags[] = {
        GPIO_OUTPUT_INACTIVE, GPIO_INPUT, GPIO_OUTPUT_ACTIVE, GPIO_OUTPUT_ACTIVE, GPIO_OUTPUT_ACTIVE, GPIO_INPUT, GPIO_INPUT
    };

    for (int i=0;i<ARRAY_SIZE(pins);++i) {
        if (!device_is_ready(pins[i]->port)) return -ENODEV;
        int r = gpio_pin_configure_dt(pins[i], flags[i]);
        if (r) return r;
    }
    return 0;
}

const struct board_gpio* board_gpio_get(void){ return &g_cfg; }

int board_led_set(bool on) { return gpio_pin_set_dt(&g_cfg.led, on); }
bool board_btn_get(void) { return !gpio_pin_get_dt(&g_cfg.btn); } /* active low */
void io_int_pulse_us(uint32_t usec){
    gpio_pin_set_dt(&g_cfg.io_int, 0);
    k_busy_wait(usec);
    gpio_pin_set_dt(&g_cfg.io_int, 1);
}
int power_sensor(bool on){ return gpio_pin_set_dt(&g_cfg.en_sensor, on); }
int power_rpu(bool on){ return gpio_pin_set_dt(&g_cfg.en_rpu, on); }
bool soh_alarm_get(void){ return gpio_pin_get_dt(&g_cfg.soh_alarm); }
bool soh_ok_get(void){ return gpio_pin_get_dt(&g_cfg.soh_ok); }
