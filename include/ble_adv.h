#pragma once
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* 간단 광고 설정: 인터벌만 사용 (ms). 나머지는 내부에서 비연결/레거시 고정 */
typedef struct
{
    uint32_t interval_ms; /* 권장: 100~10000ms */
} ble_cfg_t;

/* 인터벌 유닛 변환(0.625ms) — 헤더에 inline로 둬서 링크 에러 방지 */
static inline uint16_t ble_to_0p625(uint32_t ms)
{
    uint32_t units = (ms * 1000U) / 625U;
    if (units < 0x0020)
        units = 0x0020; /* 20ms */
    if (units > 0x4000)
        units = 0x4000; /* 10.24s */
    return (uint16_t)units;
}

const ble_cfg_t *ble_get_last_cfg(void);

/* ===================== API 구현 ===================== */
int Init_Ble(void);
int Tx_Ble(const uint8_t *mfg, size_t mfg_len); /* 브로드캐스트 시작/갱신 */

/* 모든 광고 세트/자원 중단 */
int Ble_Stop(void);

/* 마지막 설정으로 광고 시작(레거시/확장 둘 다 내부에서 처리) */
int Ble_Start(void);

/* 상태 조회용 */
bool Ble_IsRunning(void);
