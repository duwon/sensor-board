#include "ble_adv.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

/* 내부 상태 */
static bool legacy_started;
static ble_cfg_t g_last_cfg;
const ble_cfg_t *ble_get_last_cfg(void) { return &g_last_cfg; }

/* appearance: Generic Sensor(0x0340) — 스캐너에 Device type 노출용 */
static const uint16_t k_gap_appearance = 0x0340;

/* ---------- 고정 MAC(Identity) 저장/복원 ---------- */
static bt_addr_le_t g_locked_id;
static bool g_addr_loaded;

static int settings_ble_id_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (strcmp(name, "ble_id") != 0)
        return -ENOENT;
    uint8_t buf[7];
    int rc = read_cb(cb_arg, buf, MIN(len, sizeof(buf)));
    if (rc <= 0)
        return rc ? rc : -EINVAL;
    g_locked_id.type = buf[0];
    memcpy(g_locked_id.a.val, &buf[1], 6);
    g_addr_loaded = true;
    return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(app, "app", NULL, settings_ble_id_set, NULL, NULL);

static void store_ble_id(const bt_addr_le_t *id)
{
    uint8_t buf[7];
    buf[0] = id->type;
    memcpy(&buf[1], id->a.val, 6);
    (void)settings_save_one("app/ble_id", buf, sizeof(buf));
}

static void log_identity_addr(void)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t cnt = ARRAY_SIZE(addrs);
    bt_id_get(addrs, &cnt);
    for (size_t i = 0; i < cnt; i++)
    {
        char s[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addrs[i], s, sizeof(s));
        LOG_INF("BT ID[%u] %s", (unsigned)i, s);
    }
}

/* ---------- API 구현 ---------- */
int ble_init(void)
{
    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("bt_enable: %d", err);
        return err;
    }

    (void)settings_load(); /* app/ble_id 로드 시도 */

    /* 첫 부팅이면 현재 ID0를 저장 → 이후에는 항상 동일 주소 사용 */
    bt_addr_le_t cur;
    size_t cnt = 1;
    bt_id_get(&cur, &cnt);
    if (!g_addr_loaded)
    {
        g_locked_id = cur; /* 보통 static random */
        store_ble_id(&g_locked_id);
        g_addr_loaded = true;
        LOG_INF("First boot: persist current ID0");
    }

    /* 광고/연결 시작 전에 ID0를 고정 주소로 reset */
    err = bt_id_reset(0, &g_locked_id, NULL);
    if (err)
    {
        LOG_WRN("bt_id_reset failed: %d (continue)", err);
    }

    log_identity_addr();
    LOG_INF("BLE init OK");
    return 0;
}

/* 설정만 반영(레거시 비연결 광고 고정, 시작은 Tx_Ble가 담당) */
int Init_Ble(const ble_cfg_t *cfg)
{
    if (!cfg)
        return -EINVAL;

    g_last_cfg = *cfg;

    /* 현재 레거시 광고만 정지(EXT 미사용) */
    if (legacy_started)
    {
        (void)bt_le_adv_stop();
        legacy_started = false;
    }

    LOG_INF("Init_Ble: interval=%ums (legacy, non-connectable)", g_last_cfg.interval_ms);
    return 0;
}

/* 브로드캐스트 시작/갱신 — 기본 패킷만 (Flags + Appearance + Manufacturer) */
int Tx_Ble(const uint8_t *mfg, size_t mfg_len)
{
    if (!mfg || !mfg_len)
        return -EINVAL;

    const uint8_t flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
    struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags)),
        BT_DATA(BT_DATA_GAP_APPEARANCE, (const uint8_t *)&k_gap_appearance, sizeof(k_gap_appearance)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
    };

    /* 레거시, non-connectable, non-scannable */
    struct bt_le_adv_param p = BT_LE_ADV_PARAM_INIT(
        0 /* no connectable, no use_name */,
        ble_to_0p625(g_last_cfg.interval_ms),
        ble_to_0p625(g_last_cfg.interval_ms),
        NULL);

    int err;
    if (!legacy_started)
    {
        err = bt_le_adv_start(&p, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err == -EALREADY)
        {
            /* 이미 실행 중이면 데이터 갱신 */
            err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        }
        if (err)
        {
            LOG_ERR("legacy adv start/update: %d", err);
            return err;
        }
        legacy_started = true;
        LOG_INF("Advertising started (legacy, %ums)", g_last_cfg.interval_ms);
    }
    else
    {
        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err)
        {
            LOG_ERR("legacy adv update: %d", err);
            return err;
        }
        LOG_INF("Advertising payload updated");
    }
    return 0;
}

int Ble_Stop(void)
{
    if (legacy_started)
    {
        int err = bt_le_adv_stop();
        if (err)
            return err;
        legacy_started = false;
        LOG_INF("Advertising stopped");
    }
    return 0;
}
