
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define TCA9534_I2C_ADDR 0x20

struct dip_bits {
    uint8_t model:3; /* [2:0] */
    uint8_t mode_sub:1; /* [3] */
    uint8_t phy:1;  /* [4] */
    uint8_t period:1; /* [5] */
    uint8_t pwr:1;  /* [6] */
    uint8_t legacy:1; /* [7] */
};

int dip_read_u8(uint8_t *raw);
struct dip_bits parse_dip(uint8_t raw);
