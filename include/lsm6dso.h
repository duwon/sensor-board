/*
 * lsm6dso.h
 *
 * LSM6DSO 6축 IMU 센서 드라이버 (ISO 2954:2012 / 1024샘플 규격 반영)
 */
#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* FIFO 진단용 버스트 덤프 */
    struct shell;

    typedef struct
    {
        uint16_t n; /* 수집된 샘플 수 (목표: 1024) */

        /* 전체대역 통계 (m/s^2 ×100) */
        int16_t peak_ms2_x100[3];
        int16_t rms_ms2_x100[3];

        /* 10–1000 Hz 대역 제한 통계 (m/s^2 ×100) */
        int16_t bl_peak_ms2_x100[3];
        int16_t bl_rms_ms2_x100[3];

        /* 10–1000 Hz 속도 (주파수 적분 기반) mm/s ×100 */
        int16_t bl_peak_mmps_x100[3];
        int16_t bl_rms_mmps_x100[3];

        /* 디버그 */
        uint8_t whoami;
        bool wtm_reached;
    } lsm6dso_stats_t;

    typedef enum
    {
        LSM6DSO_SCALE_4G = 4,
        LSM6DSO_SCALE_16G = 16,
    } lsm6dso_scale_t;

    /* 초기화(ODR=3.33kHz, FS=±4g, FIFO=Continuous, WTM=512) */
    int lsm6dso_init(void);

    /* Active Polling으로 1024샘플(512x2) 캡처 후 통계 계산 */
    int lsm6dso_capture_once(lsm6dso_stats_t *out, lsm6dso_scale_t scale);
    int lsm6dso_capture_acc_only(lsm6dso_stats_t *out, lsm6dso_scale_t scale);
    int lsm6dso_capture_vel_only(lsm6dso_stats_t *out, lsm6dso_scale_t scale);

    typedef struct
    {
        float rms_ms2;
        float peak_ms2;
        float rms_mg;
        float peak_mg;
    } lsm6dso_psd_acc_t;

    typedef struct
    {
        float rms_mmps;
        float peak_mmps;
    } lsm6dso_psd_vel_t;

    /* 가속도 오프셋(DC 바이어스) 보정 */
    int set_calibration_lsm6dso(lsm6dso_scale_t scale);
    void clear_calibration_lsm6dso(void);

    /* 레지스터 덤프 */
    struct shell;
    int lsm6dso_dump_regs(const struct shell *shell);

#ifdef __cplusplus
}
#endif
