
#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>
#include "sensors.h"
#include "dip_switch.h"

int ble_init(void);
int ble_update_and_advertise(const struct dip_bits *dip, const sensor_sample_t *smp);
