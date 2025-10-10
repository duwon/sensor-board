
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define LTC3337_ADDR 0x64

struct ltc3337_status {
    bool bat_ok;
    uint8_t qlsb_code;
    uint32_t mah_used_fs; /* Field-scaled mAh (per spec table) */
};

int ltc3337_init(void);
int ltc3337_read_status(struct ltc3337_status *st);
