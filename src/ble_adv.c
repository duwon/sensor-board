#include "ble_adv.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <string.h>
#include "app_diag.h"

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);


/* ===================== 내부 설정 ===================== */
/* 확장 광고 파라미터: 비연결/비스캔, 공용 주소 사용, 1M PHY */
#define ADV_ONE_SHOT_MS 200      /* "1회 전송" 유사 동작 위해 매우 짧게 광고 */
#define ADV_INT_MIN_100MS 0x00A0 /* 0x00A0 * 0.625ms = 100ms */
#define ADV_INT_MAX_100MS 0x00A0

static struct bt_le_ext_adv *s_ext_adv;
static bool s_is_ext;          // 현재 모드가 확장 광고인지
static atomic_t s_adv_running = ATOMIC_INIT(1); // 실행 상태 플래그

/* 확장 광고 파라미터 (Zephyr 4.1.99 / NCS 3.1.0 호환) */
static const struct bt_le_adv_param ext_adv_param = {
    .id = BT_ID_DEFAULT,
    .sid = 0, /* 0~15, 여기선 0 사용 */
    .secondary_max_skip = 0,
    .options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY |
                BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M),
    .interval_min = ADV_INT_MIN_100MS,
    .interval_max = ADV_INT_MAX_100MS,
    .peer = NULL,
};

bool Ble_IsRunning(void)
{
    return atomic_get(&s_adv_running);
}

int Ble_Start(void)
{
    if (atomic_get(&s_adv_running))
    {
        LOG_INF("BLE advertising already running");
        return 0;
    }

    /* 마지막/현재 설정으로 컨트롤러 준비 (NULL: 내부 last_cfg 사용 가정) */
    int err = Init_Ble();
    if (err)
    {
        LOG_ERR("Init_Ble failed (%d)", err);
        return err;
    }

    LOG_INF("BLE advertising started");
    return 0;
}

int Ble_Stop(void)
{
    int err = 0;

    /* 이미 중단된 상태면 빠르게 리턴 */
    if (!atomic_get(&s_adv_running))
    {
        LOG_INF("BLE advertising already stopped");
        return 0;
    }

    if (s_is_ext)
    {
        if (s_ext_adv)
        {
            err = bt_le_ext_adv_stop(s_ext_adv);
            if (err && err != -EALREADY)
            {
                LOG_ERR("bt_le_ext_adv_stop failed (%d)", err);
                return err;
            }
        }
    }
    else
    {
        err = bt_le_adv_stop();
        if (err && err != -EALREADY)
        {
            LOG_ERR("bt_le_adv_stop failed (%d)", err);
            return err;
        }
    }

    atomic_clear(&s_adv_running);
    LOG_INF("BLE advertising stopped");
    return 0;
}

/* ===================== API 구현 ===================== */
int Init_Ble(void)
{
    int err = bt_enable(NULL);
    if (err)
    {
        printk("bt_enable failed (%d)\n", err);
        return err;
    }

    /* 확장 광고 세트 생성 (Non-connectable, Non-scannable) */
    struct bt_le_ext_adv *adv = NULL;
    err = bt_le_ext_adv_create(&ext_adv_param, NULL, &adv);
    if (err)
    {
        printk("bt_le_ext_adv_create failed (%d)\n", err);
        return err;
    }
    s_ext_adv = adv;

    printk("BLE initialized. Device name: %s\n", CONFIG_BT_DEVICE_NAME);
    return 0;
}

/* 확장 광고 한 번 짧게 켰다가 끄는 방식으로 "1회 전송" 유사 동작 수행
 * 주의: 확장 광고에서는 일반적으로 Flags AD를 포함하지 않습니다.
 * 데이터는 Manufacturer Specific Data(0xFF) 단일 AD 블록만 넣습니다.
 */
int Tx_Ble(const uint8_t *mfg, size_t mfg_len)
{
    if (!mfg || mfg_len == 0)
    {
        return -EINVAL;
    }
    if (!s_ext_adv)
    {
        return -EIO;
    }

    /* AD 구성: Manufacturer Specific Data 하나만 */
    struct bt_data ad = {
        .type = BT_DATA_MANUFACTURER_DATA, /* 0xFF */
        .data_len = (uint8_t)mfg_len,
        .data = mfg,
    };

    int err = bt_le_ext_adv_set_data(s_ext_adv, &ad, 1, NULL, 0);
    if (err)
    {
        printk("bt_le_ext_adv_set_data failed (%d)\n", err);
        return err;
    }

    /* 비연결/비스캔 광고 시작 */
    struct bt_le_ext_adv_start_param start = {
        .timeout = 0,    /* Host 타임아웃 사용 안 함 (우리가 직접 끔) */
        .num_events = 0, /* 제한 없음 */
    };

    err = bt_le_ext_adv_start(s_ext_adv, &start);
    if (err)
    {
        printk("bt_le_ext_adv_start failed (%d)\n", err);
        return err;
    }

    /* 아주 짧은 시간만 송출 */
    k_msleep(ADV_ONE_SHOT_MS);

    /* 광고 중지 */
    err = bt_le_ext_adv_stop(s_ext_adv);
    if (err)
    {
        printk("bt_le_ext_adv_stop failed (%d)\n", err);
        return err;
    }
    if (diag_on())
    {
        printk("EXT ADV burst done (%u ms, MFG %u bytes)\n", (unsigned)ADV_ONE_SHOT_MS, (unsigned)mfg_len);
    }
    return 0;
}
