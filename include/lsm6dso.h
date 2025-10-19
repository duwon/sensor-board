#pragma once
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 스케일 선택 */
typedef enum {
    LSM6DSO_FS_4G  = 0,  /* default */
    LSM6DSO_FS_16G = 1,
} lsm6dso_fs_t;

typedef struct {
    uint16_t n;                 /* 수집된 유효 샘플 수(축 동일) */

    /* 시간영역 간이 통계 (전체대역) : m/s^2 ×100 고정소수 */
    int16_t peak_ms2_x100[3];
    int16_t rms_ms2_x100[3];

    /* 대역 제한(10–1000 Hz) 통계 : m/s^2 ×100 고정소수 */
    int16_t bl_peak_ms2_x100[3];
    int16_t bl_rms_ms2_x100[3];

    /* 디버그 */
    uint8_t  whoami;
    bool     wtm_reached;
} lsm6dso_stats_t;

/* 초기화(전원 인가 후 50ms 이상 지연 뒤 호출 권장) */
int lsm6dso_init(lsm6dso_fs_t fs);

/* 단일 버스트 측정: FIFO 0.3s 채움 → 999워드 도달 폴링 → 6993B 일괄 읽기 → 통계 산출 */
int lsm6dso_capture_once(lsm6dso_stats_t *out);

/* 내부 사용: m/s^2 스케일 팩터(사양서 값 기반) */
float lsm6dso_scale_ms2_per_lsb(lsm6dso_fs_t fs);

#ifdef __cplusplus
}
#endif
