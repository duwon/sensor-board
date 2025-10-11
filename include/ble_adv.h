#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"
#include "dip_switch.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* 기존 구조체/열거형은 그대로 사용한다고 가정 */
    typedef enum
    {
        PHY_1M = 1,
        PHY_2M = 2,
        PHY_CODED = 3
    } phy_t;

    typedef struct
    {
        bool legacy;          /* true=Legacy, false=EXT */
        uint32_t interval_ms; /* adv interval (ms) */
        phy_t phy;            /* PHY 요청 (EXT는 무시될 수 있음) */
        bool connectable;     /* 본 프로젝트는 항상 false */
    } ble_cfg_t;

    /* 기존 루프용 (그대로 유지) */
    int ble_init(void);
    int ble_update_and_advertise(const struct dip_bits *dip, const sensor_sample_t *smp);

    /* ===== 신규 API(요구사항) ===== */
    int Init_Ble(const ble_cfg_t *cfg);

    /* Tx_Ble: Manufacturer(AD)로 브로드캐스팅 시작/갱신 */
    int Tx_Ble(const uint8_t *mfg, size_t mfg_len);

    /* Rx_Ble: Scan Response 페이로드 등록(스캔요청 오면 이 데이터로 응답)
       - len=0 이면 스캔응답 사용 안 함(순수 NON-SCANNABLE) */
    int Rx_Ble(const uint8_t *sr, size_t len);

    /* 광고 정지(슬립 직전에 호출) */
    int Ble_Stop(void);

    /* (선택) 마지막 BLE 설정 조회 */
    const ble_cfg_t *ble_get_last_cfg(void);

#ifdef __cplusplus
}
#endif