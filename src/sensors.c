/* sensors.c */
#include "sensors.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include "ntc.h"
#include "xgzp6897d.h"
#include "xgzp6847d.h"
#include "lsm6dso.h"
#include "ssc_pressure.h"

/** @file sensors.c
 * @brief 보드에 연결된 각종 센서(ADC, I2C)를 제어하고 값을 읽는 함수 구현.
 */

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);
lsm6dso_stats_t g_lsm6dso_stats; /* IMU 통계 글로벌 */

/** @brief 센서 공통 초기화
 *
 * @retval 0 성공
 * @retval -ENOTSUP ADC가 비활성화된 경우
 * @retval 음수값 ADC 채널 설정 실패 시 Zephyr 에러 코드
 */
int Init_Sensor(void)
{
    int ret = 0;

    ret = Init_ntc();

    return 0;
}

/**
 * @brief 센서 ID에 따라 값을 읽어온다.
 *
 * @param sensor_id sensors.h에 정의된 센서 ID
 * @return 센서 값(단위별 x100 정수) 또는 음수 에러코드
 */
int32_t Get_Sensor_Value(uint8_t sensor_id)
{
    /* 공통 변환 상수 */
    const float PA_PER_MMH2O = 9.80665f;     /* 1 mmH2O = 9.80665 Pa */
    const float PA_PER_BAR   = 100000.0f;    /* 1 bar = 100000 Pa */

    switch (sensor_id & 0x0F)
    {
    /* ---------------- Temperature (returns °C x100) ---------------- */
    case SENSOR_ID_TEMPERATURE_NTC_2:  /* 온도 (10k NTC) */
    case SENSOR_ID_TEMPERATURE_NTC_1:  /* 온도 (10k NTC) */
    {
        int16_t ntc_c_x100 = 0;
        (void)read_ntc(&ntc_c_x100);
        return (int32_t)ntc_c_x100; /* 온도는 °C x100 */
    }

    /* ---------------- Pressure: Air Flow ---------------- */
    case SENSOR_ID_PRESSURE_AIR_FLOW_XGZP6897_001K: /* 압력(Air Flow) : XGZP6897D001KPDPN → Pa */
    {
        float p_pa = 0.0f;
        (void)read_xgzp6897_filtered(XGZP6897_RANGE_001K, &p_pa, NULL, true);
        float mmh2o_x100 = (p_pa / PA_PER_MMH2O) * 100.0f; /* mmH2O x100 */
        return (int32_t)mmh2o_x100;
    }
    case SENSOR_ID_PRESSURE_AIR_FLOW_SSCDJNN002ND:  /* 압력(Air Flow) : SSCDJNN002ND2A3 → mmH2O */
    {
        float p_mmh2o = 0.0f;
        (void)read_ssc_filtered(SSCDJNN002ND2A3, &p_mmh2o, NULL, true);
        float mmh2o_x100 = p_mmh2o * 100.0f; /* mmH2O x100 */
        return (int32_t)mmh2o_x100;
    }

    /* ---------------- Pressure: XGZP6897 010K (Inlet/Outlet 공용) ---------------- */
    case SENSOR_ID_PRESSURE_INLET_XGZP6897_010K:
    case SENSOR_ID_PRESSURE_OUTLET_XGZP6897_010K:
    {
        float p_pa = 0.0f;
        (void)read_xgzp6897_filtered(XGZP6897_RANGE_010K, &p_pa, NULL, true);
        float mmh2o_x100 = (p_pa / PA_PER_MMH2O) * 100.0f; /* mmH2O x100 */
        return (int32_t)mmh2o_x100;
    }

    /* ---------------- Pressure: SSC 100MD (Inlet/Outlet 공용) ---------------- */
    case SENSOR_ID_PRESSURE_INLET_SSCDJNN100MD:
    case SENSOR_ID_PRESSURE_OUTLET_SSCDJNN100MD:
    {
        float p_mmh2o = 0.0f;
        (void)read_ssc_filtered(SSCDJNN100MD2A3, &p_mmh2o, NULL, true);
        float mmh2o_x100 = p_mmh2o * 100.0f; /* mmH2O x100 */
        return (int32_t)mmh2o_x100;
    }

    /* ---------------- Pressure: Air Header ---------------- */
    case SENSOR_ID_PRESSURE_AIR_HEADER_XGZP6847_001MP: /* 압력(Air Header) : XGZP6847D001MPGPN → Pa */
    {
        float p_pa = 0.0f;
        (void)read_xgzp6847_filtered(XGZP6847_RANGE_001MPGPN, &p_pa, NULL, true);
        float bar_x100 = (p_pa / PA_PER_BAR) * 100.0f; /* bar x100 */
        return (int32_t)bar_x100;
    }
    case SENSOR_ID_PRESSURE_AIR_HEADER_SSCDJNN010BA:  /* 압력(Air Header) : SSCDJNN010BA2A3 → bar */
    {
        float p_bar = 0.0f;
        (void)read_ssc_filtered(SSCDJNN010BA2A3, &p_bar, NULL, true);
        float bar_x100 = p_bar * 100.0f; /* bar x100 */
        return (int32_t)bar_x100;
    }

    default:
        return -ENOTSUP;
    }
}

/**
 * @brief IMU(LSM6DSO) 가속도/속도 값을 캡처하고 전역 변수에 저장
 *
 * @param sensor_id IMU 전용 센서 ID (가속도/속도, 4g/16g) @ref sensors.h
 * @return 0 on success, 음수 에러코드 on failure
 */
int16_t Get_Imu_Value(uint8_t sensor_id)
{
    lsm6dso_stats_t st = {0};
    int rc = 0;

    switch (sensor_id)
    {
    case SENSOR_ID_LSM6DSO_ACCEL_4G:
        rc = lsm6dso_capture_acc_only(&st, LSM6DSO_SCALE_4G);
        break;
    case SENSOR_ID_LSM6DSO_ACCEL_16G:
        rc = lsm6dso_capture_acc_only(&st, LSM6DSO_SCALE_16G);
        break;
    case SENSOR_ID_LSM6DSO_VELO_4G:
        rc = lsm6dso_capture_vel_only(&st, LSM6DSO_SCALE_4G);
        break;
    case SENSOR_ID_LSM6DSO_VELO_16G:
        rc = lsm6dso_capture_vel_only(&st, LSM6DSO_SCALE_16G);
        break;
    default:
        return -ENOTSUP;
    }

    if (rc == 0)
    {
        g_lsm6dso_stats = st;
    }

    return (int16_t)rc;
}


/**
 * @brief 센서별 캘리브레이션(오프셋) 설정
 *
 * @param sensor_id 센서 ID
 * @return 0 on success, 음수 에러코드 on failure
 */
int Set_Calibration(uint8_t sensor_id)
{
    switch (sensor_id)
    {
    /* XGZP6897D 001K (Air Flow) */
    case SENSOR_ID_PRESSURE_AIR_FLOW_XGZP6897_001K:
        return set_calibration_xgzp6897(XGZP6897_RANGE_001K);

    /* XGZP6897D 010K (Inlet/Outlet) */
    case SENSOR_ID_PRESSURE_INLET_XGZP6897_010K:
    case SENSOR_ID_PRESSURE_OUTLET_XGZP6897_010K:
        return set_calibration_xgzp6897(XGZP6897_RANGE_010K);

    /* XGZP6847D 001MP (Air Header) */
    case SENSOR_ID_PRESSURE_AIR_HEADER_XGZP6847_001MP:
        return set_calibration_xgzp6847(XGZP6847_RANGE_001MPGPN);

    /* SSC Honeywell models */
    case SENSOR_ID_PRESSURE_AIR_FLOW_SSCDJNN002ND:
        return set_calibration_ssc(SSCDJNN002ND2A3);
    case SENSOR_ID_PRESSURE_INLET_SSCDJNN100MD:
        return set_calibration_ssc(SSCDJNN100MD2A3);
    case SENSOR_ID_PRESSURE_OUTLET_SSCDJNN100MD:
        return set_calibration_ssc(SSCDJNN100MD2A3);
    case SENSOR_ID_PRESSURE_AIR_HEADER_SSCDJNN010BA:
        return set_calibration_ssc(SSCDJNN010BA2A3);

    /* LSM6DSO IMU (가속도/속도 공통 오프셋) */
    case SENSOR_ID_LSM6DSO_ACCEL_4G:
    case SENSOR_ID_LSM6DSO_VELO_4G:
        return set_calibration_lsm6dso(LSM6DSO_SCALE_4G); 
    case SENSOR_ID_LSM6DSO_ACCEL_16G:
    case SENSOR_ID_LSM6DSO_VELO_16G:
        return set_calibration_lsm6dso(LSM6DSO_SCALE_16G);

    /* Temperature NTC: not applicable */
    case SENSOR_ID_TEMPERATURE_NTC_1:
    case SENSOR_ID_TEMPERATURE_NTC_2:
    default:
        return -ENOTSUP;
    }
}

/**
 * @brief 센서별 캘리브레이션(오프셋) 해제
 *
 * @param sensor_id 센서 ID
 * @return 0 on success, 음수 에러코드 on failure
 */
int Clear_Calibration(uint8_t sensor_id)
{
    switch (sensor_id)
    {
    /* XGZP6897D ranges share one offset store in driver */
    case SENSOR_ID_PRESSURE_AIR_FLOW_XGZP6897_001K:
    case SENSOR_ID_PRESSURE_INLET_XGZP6897_010K:
    case SENSOR_ID_PRESSURE_OUTLET_XGZP6897_010K:
        clear_calibration_xgzp6897();
        return 0;

    /* XGZP6847D 001MP */
    case SENSOR_ID_PRESSURE_AIR_HEADER_XGZP6847_001MP:
        clear_calibration_xgzp6847();
        return 0;

    /* SSC Honeywell models */
    case SENSOR_ID_PRESSURE_AIR_FLOW_SSCDJNN002ND:
    case SENSOR_ID_PRESSURE_INLET_SSCDJNN100MD:
    case SENSOR_ID_PRESSURE_OUTLET_SSCDJNN100MD:
    case SENSOR_ID_PRESSURE_AIR_HEADER_SSCDJNN010BA:
        clear_calibration_ssc();
        return 0;
    /* LSM6DSO IMU (가속도/속도 공통 오프셋) */
    case SENSOR_ID_LSM6DSO_ACCEL_4G:
    case SENSOR_ID_LSM6DSO_ACCEL_16G:
    case SENSOR_ID_LSM6DSO_VELO_4G:
    case SENSOR_ID_LSM6DSO_VELO_16G:
        clear_calibration_lsm6dso();
        return 0;

    /* Temperature NTC: not applicable */
    case SENSOR_ID_TEMPERATURE_NTC_1:
    case SENSOR_ID_TEMPERATURE_NTC_2:
    default:
        return -ENOTSUP;
    }
}
