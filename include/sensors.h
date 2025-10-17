
#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>


/* 센서 공통 초기화 (ADC 채널 준비 등). 0이면 OK */
int sensors_init(void);

/* VDD (mV) 측정: SAADC VDD/4 채널 사용 */
int read_vdd_mv(int16_t *vdd_mv);

/* NTC 온도 읽기: 섭씨 × 100 단위로 반환 (예: 25.34°C → 2534) */
int read_ntc_ain1_cx100(int16_t *cx100);

typedef struct {
    int32_t p_value_x100; /* pressure * 100 (mmH2O or mbar depends on mode) */
    int16_t temperature_c_x100; /* for NTC or sensor temp */
    uint8_t battery_pc;
    int16_t acc_rms_x100[3]; /* optional */
    int16_t acc_peak_x100[3]; /* optional */
} sensor_sample_t;


int read_pressure_0x28(sensor_sample_t *out);

