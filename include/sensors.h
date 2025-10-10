
#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>

typedef struct {
    int32_t p_value_x100; /* pressure * 100 (mmH2O or mbar depends on mode) */
    int16_t temperature_c_x100; /* for NTC or sensor temp */
    uint8_t battery_pc;
    int16_t acc_rms_x100[3]; /* optional */
    int16_t acc_peak_x100[3]; /* optional */
} sensor_sample_t;

int sensors_init(void);
int read_pressure_0x28(sensor_sample_t *out);
int read_ntc_ain1_cx100(int16_t *cx100);
