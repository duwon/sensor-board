#include "ble_adv.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

/* =========================
 * 광고 모드 선택 스위치
 * =========================
 * 1 = 레거시 광고(권장: 먼저 가시성 확보)
 * 0 = 확장 광고(옵션 B: Short Name 추가 등 가시성 보강)
 */
#define FORCE_LEGACY   1

/* 공통: 샘플 → 간단 Manufacturer payload(회사ID+ver+model+status+err+mcuT+battery+presence+V1=~13B) */
static int build_legacy_packet(uint8_t *mfg, size_t *mfg_len, const sensor_sample_t *smp)
{
    uint8_t p[20] = {0};
    int idx = 0;

    p[idx++] = 0xFF; p[idx++] = 0xFF;       /* Company ID (test) */
    p[idx++] = 0x01;                        /* Struct ver */
    p[idx++] = 0x01;                        /* Model */
    p[idx++] = 0x00;                        /* Status */
    p[idx++] = 0x00;                        /* Error  */
    p[idx++] = 25;                          /* MCU temp (dummy) */
    p[idx++] = smp->battery_pc;             /* Battery */
    p[idx++] = 0x01;                        /* Presence: only v1 */

    int32_t v1 = smp->p_value_x100;         /* 첫 값만 */
    p[idx++] = (uint8_t)(v1      );
    p[idx++] = (uint8_t)(v1 >>  8);
    p[idx++] = (uint8_t)(v1 >> 16);
    p[idx++] = (uint8_t)(v1 >> 24);

    memcpy(mfg, p, idx);
    *mfg_len = idx;
    return 0;
}

/* ===== 옵션 B(확장 광고)용: 기존 확장 광고 빌더 ===== */
static int build_ext_packet(uint8_t *mfg, size_t *mfg_len, const sensor_sample_t *smp)
{
    uint8_t p[40] = {0};
    int idx = 0;
    p[idx++] = 0xFF; p[idx++] = 0xFF;       /* Company ID (test) */
    p[idx++] = 0x01;                        /* Struct ver */
    p[idx++] = 0x01;                        /* Model */
    p[idx++] = 0x00;                        /* Status */
    p[idx++] = 0x00;                        /* Error  */
    p[idx++] = 25;                          /* MCU temp(dummy) */
    p[idx++] = smp->battery_pc;             /* Battery */
    p[idx++] = 0x01;                        /* Presence: value1 only */
    int32_t v[6] = {smp->p_value_x100, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; i++) {
        p[idx++] = (uint8_t)(v[i]      );
        p[idx++] = (uint8_t)(v[i] >>  8);
        p[idx++] = (uint8_t)(v[i] >> 16);
        p[idx++] = (uint8_t)(v[i] >> 24);
    }
    memcpy(mfg, p, idx);
    *mfg_len = idx;
    return 0;
}

/* ===== 옵션 A(레거시 광고) 상태 ===== */
static bool legacy_started;

/* ===== 옵션 B(확장 광고) 상태 ===== */
static struct bt_le_ext_adv *ext_adv;
static bool ext_started;

/* 공통 초기화: 블루투스 스택만 켜기 (모드별로 나눠 시작) */
int ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable: %d", err);
        return err;
    }
    LOG_INF("BLE init OK");

#if !FORCE_LEGACY
    /* 옵션 B: 확장 광고 핸들 생성 (Short Name 추가로 가시성 개선) */
    struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_EXT_ADV, /* EXT, non-connectable */
        160, 160, NULL
    );
    err = bt_le_ext_adv_create(&param, NULL, &ext_adv);
    if (err) {
        LOG_ERR("ext_adv_create: %d", err);
        return err;
    }
#endif

    return 0;
}

/* 메인 루프에서 주기적으로 호출 */
int ble_update_and_advertise(const struct dip_bits *dip, const sensor_sample_t *smp)
{
    const uint8_t flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;

#if FORCE_LEGACY
    /* =========================
     * 옵션 A: 레거시 광고
     * ========================= */
    uint8_t mfg[24]; size_t mfg_len = 0;
    build_legacy_packet(mfg, &mfg_len, smp);

    /* 레거시 AD: Flags + Manufacturer (<=31B) */
    struct bt_data ad[2] = {
        BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
    };

    /* 이름을 함께 내보내고 싶다면: CONFIG_BT_DEVICE_NAME 이용 (옵션)
       struct bt_data ad[3] = {
         BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags)),
         BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, strlen(CONFIG_BT_DEVICE_NAME)),
         BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
       };
       (31B 넘지 않도록 주의)
    */

    int err;
    if (!legacy_started) {
        /* Legacy non-connectable, 100ms */
        struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
            BT_LE_ADV_OPT_USE_NAME, /* 장치 이름 같이 싣고 싶으면 유지 */
            160, 160, NULL
        );
        err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err == -EALREADY) {
            LOG_WRN("legacy adv already started");
            legacy_started = true;
            /* 이미 시작되어 있었다면 데이터만 갱신 시도 */
            err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        }
        if (err) {
            LOG_ERR("legacy adv start/update: %d", err);
            return err;
        }
        legacy_started = true;
        LOG_INF("Legacy advertising active (100ms, payload=%uB)", (unsigned)mfg_len);
    } else {
        /* 실행 중이면 payload만 갱신 */
        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            LOG_ERR("legacy adv update: %d", err);
            return err;
        }
        LOG_INF("Legacy advertising payload updated");
    }
    return 0;

#else
    /* =========================
     * 옵션 B: 확장 광고
     *  - Shortened Local Name 추가
     *  - 필요 시 PHY 명시
     * ========================= */
    uint8_t mfg[40]; size_t mfg_len = 0;
    build_ext_packet(mfg, &mfg_len, smp);

    /* 이름을 짧게 실어 목록 노출을 유도 */
    const char *name = "ARX-T100"; /* 너무 길면 잘릴 수 있음 */
    struct bt_data ad[3] = {
        BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags)),
        BT_DATA(BT_DATA_NAME_SHORTENED, name, strlen(name)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
    };

    int err = bt_le_ext_adv_set_data(ext_adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("ext set_data: %d", err);
        return err;
    }

    if (!ext_started) {
        /* (선택) PHY를 1M/2M로 명시해 호환성 확보 */
        // bt_le_ext_adv_set_phy(ext_adv, BT_HCI_LE_PHY_1M, BT_HCI_LE_PHY_2M);

        err = bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
        if (err && err != -EALREADY) {
            LOG_ERR("ext adv start: %d", err);
            return err;
        }
        ext_started = true;
        LOG_INF("EXT advertising active (100ms, payload=%uB)", (unsigned)mfg_len);
    } else {
        LOG_INF("EXT advertising payload updated");
    }
    return 0;
#endif
}
