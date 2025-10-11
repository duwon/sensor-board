#include "ble_adv.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>   // settings_load(), settings_save_one
#include <zephyr/sys/byteorder.h>       // sys_cpu_to_le16
#include <zephyr/sys/util.h>            // MIN()
#include <string.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

/* ===== 내부 상태 ===== */
static bool legacy_started;
static struct bt_le_ext_adv *ext_adv;
static bool ext_started;

/* 마지막 설정 저장 */
static ble_cfg_t g_last_cfg;
const ble_cfg_t *ble_get_last_cfg(void) { return &g_last_cfg; }

/* Scan Response 버퍼(최대 31B 권장) */
static uint8_t g_scan_rsp[31];
static size_t  g_scan_rsp_len;

/* appearance 값 (Generic Sensor = 0x0340) */
static const uint16_t k_gap_appearance = 0x0340;  /* Generic Sensor */

/* ===== MAC 고정용(저장/복원) ===== */
static bt_addr_le_t g_locked_id;   /* 저장/적용용 */
static bool g_addr_loaded;

/* ───────── adv interval helper (ms -> 0.625ms unit) ─────────
 * BLE 광고 인터벌은 0.625ms 단위. 유효 범위를 벗어나지 않도록 클램핑.
 * - 최소: 0x0020 (20 * 0.625ms = 12.5ms)   *실제 제품은 더 길게 사용*
 * - 최대: 0x4000 (16384 * 0.625ms = 10240ms = 10s)
 */
static inline uint16_t to_0p625(uint32_t ms)
{
    /* 1000us/625us = 1.6 → 정수 산술로 (ms * 1000) / 625 사용 */
    uint32_t units = (ms * 1000U) / 625U;

    if (units < 0x0020U) units = 0x0020U;
    if (units > 0x4000U) units = 0x4000U;
    return (uint16_t)units;
}

/* Static Random 규격 보정: type=RANDOM & 상위 2비트=11b */
static inline void make_static_random(bt_addr_le_t *a)
{
    a->type = BT_ADDR_LE_RANDOM;
    a->a.val[5] |= 0xC0;  /* MSB 2bits = 11b (static random) */
}

/* settings 콜백: "app/ble_id"에 7바이트(AddrType + 6B addr) 저장/복원 */
static int settings_ble_id_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg)
{
    if (strcmp(name, "ble_id") != 0) return -ENOENT;

    uint8_t buf[7];
    int rc = read_cb(cb_arg, buf, MIN(len, sizeof(buf)));
    if (rc <= 0) return rc ? rc : -EINVAL;

    g_locked_id.type = buf[0];
    memcpy(g_locked_id.a.val, &buf[1], 6);

    /* 저장된 주소가 public이거나 non-static random이면 static random으로 보정 */
    make_static_random(&g_locked_id);
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
    bt_id_get(addrs, &cnt);  /* 두번째 인자는 포인터! */

    for (size_t i = 0; i < cnt; i++) {
        char s[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addrs[i], s, sizeof(s));
        LOG_INF("BT ID[%u] %s", (unsigned)i, s);
    }

    LOG_INF("Privacy: %s", IS_ENABLED(CONFIG_BT_PRIVACY) ? "ENABLED" : "DISABLED");
}


int ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable: %d", err);
        return err;
    }

    /* 저장 로드 (app/ble_id) */
    (void)settings_load();

    /* 현재 ID0 주소 확인 */
    bt_addr_le_t cur;
    size_t cnt = 1;
    bt_id_get(&cur, &cnt);

    if (!g_addr_loaded) {
        /* 첫 부팅: 현재 ID0를 기반으로 static random을 만들어 저장 */
        g_locked_id = cur;
        make_static_random(&g_locked_id);
        store_ble_id(&g_locked_id);
        g_addr_loaded = true;
        LOG_INF("First boot: persist current ID0 as static-random");
    }

    /* 항상 ID0를 고정 주소로 reset (광고/스캔 시작 전이므로 안전) */
    err = bt_id_reset(0, &g_locked_id, NULL);
    if (err) {
        LOG_WRN("bt_id_reset failed: %d (continue)", err);
    }

    log_identity_addr();  /* 확인용 로그 */

    LOG_INF("BLE init OK");
    return 0;
}


/* 설정만 반영(시작은 Tx_Ble가 담당) - "현재 모드만" stop/update */
int Init_Ble(const ble_cfg_t *cfg)
{
    if (!cfg) return -EINVAL;

    /* 정책: 브로드캐스터 고정 */
    g_last_cfg = *cfg;
    g_last_cfg.connectable = false;

    /* 1) 현재 모드만 정지 */
    if (cfg->legacy) {
        /* 레거시만 정지 */
        if (legacy_started) {
            (void)bt_le_adv_stop();
            legacy_started = false;
        }
        /* EXT 쪽은 건드리지 않음 */
    } else {
        /* EXT만 정지 */
        if (ext_started && ext_adv) {
            (void)bt_le_ext_adv_stop(ext_adv);
            ext_started = false;
        }
        /* 레거시 쪽은 건드리지 않음 */
    }

    /* 2) EXT 모드라면 핸들만 준비(시작은 안 함) */
    if (!g_last_cfg.legacy) {
        struct bt_le_adv_param p = BT_LE_ADV_PARAM_INIT(
            BT_LE_ADV_OPT_EXT_ADV,
            to_0p625(g_last_cfg.interval_ms),
            to_0p625(g_last_cfg.interval_ms),
            NULL);

        int err;
        if (!ext_adv) {
            err = bt_le_ext_adv_create(&p, NULL, &ext_adv);
            if (err) {
                LOG_ERR("ext create: %d", err);
                return err;
            }
        } else {
            err = bt_le_ext_adv_update_param(ext_adv, &p);
            if (err) {
                LOG_ERR("ext update: %d", err);
                return err;
            }
        }
        /* (필요 시) PHY 지정은 SDK 버전에 맞춰 별도로 설정 */
    }

    LOG_INF("Init_Ble: legacy=%u int=%ums phy=%u conn=0",
            g_last_cfg.legacy, g_last_cfg.interval_ms, g_last_cfg.phy);
    return 0;
}

/* 스캔응답 등록: len=0 → 미사용 */
int Rx_Ble(const uint8_t *sr, size_t len)
{
    if (!sr || len==0) {
        g_scan_rsp_len = 0;
        return 0;
    }
    if (len > sizeof(g_scan_rsp)) return -EMSGSIZE;
    memcpy(g_scan_rsp, sr, len);
    g_scan_rsp_len = len;
    return 0;
}

/* 브로드캐스팅 시작/갱신 (AD = Flags + Manufacturer) */
/* 브로드캐스팅 시작/갱신 (AD = Flags + Appearance + Manufacturer) */
int Tx_Ble(const uint8_t *mfg, size_t mfg_len)
{
    if (!mfg || !mfg_len) return -EINVAL;

    /* ★ Appearance는 LE 16-bit로 보내야 스캐너에서 타입이 보인다 */
    uint16_t appearance_le = sys_cpu_to_le16(k_gap_appearance);

    if (g_last_cfg.legacy) {
        /* ===== Legacy only: bt_le_adv_*만 사용 ===== */
        const uint8_t flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;

        struct bt_data ad[] = {
            BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags)),
            BT_DATA(BT_DATA_GAP_APPEARANCE, (const uint8_t *)&appearance_le,
                    sizeof(appearance_le)),                 /* Device type */
            BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
        };

        /* 선택: SR에 짧은 이름 등 넣고 싶으면 g_scan_rsp 사용 */
        struct bt_data sr[1];
        const struct bt_data *sr_ptr = NULL;
        size_t sr_cnt = 0;
        if (g_scan_rsp_len) {
            sr[0].type = BT_DATA_NAME_SHORTENED;
            sr[0].data = g_scan_rsp;
            sr[0].data_len = g_scan_rsp_len;
            sr_ptr = sr;
            sr_cnt = 1;
        }

        if (!legacy_started) {
            struct bt_le_adv_param p = BT_LE_ADV_PARAM_INIT(
                0,  /* non-connectable broadcaster */
                to_0p625(g_last_cfg.interval_ms),
                to_0p625(g_last_cfg.interval_ms),
                NULL);

            int err = bt_le_adv_start(&p, ad, ARRAY_SIZE(ad), sr_ptr, sr_cnt);
            if (err == -EALREADY) {
                err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sr_ptr, sr_cnt);
            }
            if (err) { LOG_ERR("legacy start/update: %d", err); return err; }
            legacy_started = true;
        } else {
            int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sr_ptr, sr_cnt);
            if (err) { LOG_ERR("legacy update: %d", err); return err; }
        }
        return 0;

    } else {
        /* ===== EXT only: bt_le_ext_adv_*만 사용 ===== */
        if (!ext_adv) {
            /* 방어: Init_Ble()에서 ext_adv를 만들어 두는 정책 */
            LOG_ERR("ext_adv is NULL (Init_Ble must prepare it)");
            return -EINVAL;
        }

        struct bt_data ad[] = {
            /* Flags는 EXT에서 무의미하지만, 일부 스캐너 호환 때문에 생략/유지는 선택 */
            BT_DATA(BT_DATA_GAP_APPEARANCE, (const uint8_t *)&appearance_le,
                    sizeof(appearance_le)),                 /* Device type */
            BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
        };

        int err = bt_le_ext_adv_set_data(ext_adv, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) { LOG_ERR("ext set_data: %d", err); return err; }

        if (!ext_started) {
            err = bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
            if (err == -EALREADY) {
                /* 이미 돌고 있으면 OK */
            } else if (err) {
                LOG_ERR("ext start: %d", err);
                return err;
            }
            ext_started = true;
        }
        return 0;
    }
}


int Ble_Stop(void)
{
    int err = 0;

    /* 현재 설정 확인 */
    const ble_cfg_t *cfg = ble_get_last_cfg();
    if (cfg && cfg->legacy) {
        if (legacy_started) {
            int e = bt_le_adv_stop();
            if (!err) err = e;
            legacy_started = false;
        }
    } else {
        if (ext_started && ext_adv) {
            int e = bt_le_ext_adv_stop(ext_adv);
            if (!err) err = e;
            ext_started = false;
        }
    }
    return err;
}
