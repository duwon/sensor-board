#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>

/* ------------------------------------------------------
 * Sensor ID Definitions (파트명 → 숫자 치환)
 * ------------------------------------------------------ */
/* 온도 (10k NTC) */
#define SENSOR_ID_TEMPERATURE_NTC_2 3
#define SENSOR_ID_TEMPERATURE_NTC_1 11

/* 압력 (Air Flow) */
#define SENSOR_ID_PRESSURE_AIR_FLOW_SSCDJNN002ND 4   /* SSCDJNN002ND2A3  ±50.8 mmH2O */
#define SENSOR_ID_PRESSURE_AIR_FLOW_XGZP6897_001K 12 /* XGZP6897D001KPDPN ±100 mmH2O */

/* 압력 (Air Header) */
#define SENSOR_ID_PRESSURE_AIR_HEADER_SSCDJNN010BA 5    /* SSCDJNN010BA2A3  0 ~ 10 bar */
#define SENSOR_ID_PRESSURE_AIR_HEADER_XGZP6847_001MP 13 /* XGZP6847D001MPGPN -1 ~ 10 bar */

/* 압력 (Inlet) */
#define SENSOR_ID_PRESSURE_INLET_SSCDJNN100MD 7   /* SSCDJNN100MD2A3  ±1020 mmH2O */
#define SENSOR_ID_PRESSURE_INLET_XGZP6897_010K 15 /* XGZP6897D010KPDPN ±1000 mmH2O */

/* 압력 (Outlet) */
#define SENSOR_ID_PRESSURE_OUTLET_SSCDJNN100MD 6   /* SSCDJNN100MD2A3  ±1020 mmH2O */
#define SENSOR_ID_PRESSURE_OUTLET_XGZP6897_010K 14 /* XGZP6897D010KPDPN ±1000 mmH2O */



/** @struct sensor_sample_t
 * @brief 여러 센서 측정값을 통합하여 저장하는 구조체
 */
typedef struct
{
    int32_t p_value_x100;       /**< 압력 값 * 100 (단위는 모드에 따라 mmH2O 또는 mbar) */
    int16_t temperature_c_x100; /**< 온도 값 * 100 (NTC 또는 센서 자체 온도) */
    uint8_t battery_pc;         /**< 배터리 잔량 (%) */
    int16_t acc_rms_x100[3];    /**< 가속도 RMS * 100 (선택적) */
    int16_t acc_peak_x100[3];   /**< 가속도 Peak * 100 (선택적) */
} sensor_sample_t;

/** @file sensors.h
 * @brief 센서 인터페이스 정의 파일
 */

/** @brief 센서 공통 초기화 (ADC 채널 준비 등).
 * @see Init_Sensor
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int Init_Sensor(void);

int32_t Get_Sensor_Value(uint8_t sensor_id);
int16_t Get_Imu_Value(uint8_t sensor_id);
/* Calibration controls */
int Set_Calibration(uint8_t sensor_id);
int Clear_Calibration(uint8_t sensor_id);
