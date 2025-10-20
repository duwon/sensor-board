#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        LSM6DSO_FS_4G = 0,
        LSM6DSO_FS_16G = 1,
    } lsm6dso_fs_t;

    typedef struct
    {
        uint16_t n; /* 수집된 샘플 수 */

        /* 전체대역 통계 (m/s^2 ×100 고정소수) */
        int16_t peak_ms2_x100[3];
        int16_t rms_ms2_x100[3];

        /* 10–1000 Hz 대역 제한 통계 (m/s^2 ×100) */
        int16_t bl_peak_ms2_x100[3];
        int16_t bl_rms_ms2_x100[3];

        /* 디버그 */
        uint8_t whoami;
        bool wtm_reached;
    } lsm6dso_stats_t;

    /* 초기화(ODR=3.33kHz, FS=±4g, Gyro OFF, FIFO=Continuous로 세팅) */
    int lsm6dso_init(lsm6dso_fs_t fs);

    /* 0.33s 동안 DRDY 폴링으로 직접 캡처 + 통계 계산 */
    int lsm6dso_capture_once(lsm6dso_stats_t *out);

    /* LSB→m/s^2 스케일 팩터 */
    float lsm6dso_scale_ms2_per_lsb(lsm6dso_fs_t fs);

    /* 레지스터 덤프(옵션): 쉘에서 상태 확인용 */
    struct shell;
    int lsm6dso_dump_regs(const struct shell *shell);

#ifdef __cplusplus
}
#endif
