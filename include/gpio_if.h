
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

struct board_gpio {
    const struct gpio_dt_spec led;
    const struct gpio_dt_spec btn;
    const struct gpio_dt_spec io_int;
    const struct gpio_dt_spec en_sensor;
    const struct gpio_dt_spec en_rpu;
    const struct gpio_dt_spec soh_alarm;
    const struct gpio_dt_spec soh_ok;
};

int board_gpio_init(void);
const struct board_gpio* board_gpio_get(void);

void board_led_set(bool on);
bool board_btn_get(void);
void io_int_pulse_us(uint32_t usec);
void power_sensor(bool on);
void power_rpu(bool on);
bool soh_alarm_get(void);
bool soh_ok_get(void);
