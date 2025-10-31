#include "ble_adv.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

/** @brief 1회 광고 지속 시간 (200ms) */
#define ADV_ONE_SHOT_MS K_MSEC(200)
/** @brief 최소 광고 인터벌 (100ms = 0x00A0 * 0.625ms) */
#define ADV_INT_MIN 0x00A0
/** @brief 최대 광고 인터벌 (100ms = 0x00A0 * 0.625ms) */
#define ADV_INT_MAX 0x00A0
/** @brief 스캔 응답 지속 시간 5분 */
#define SCAN_RSP_DURATION_MS (5UL * 60UL * 100UL)

/** @brief 확장 광고 파라미터 구조체
 *
 * 비연결(Non-connectable), 비검색(Non-scannable) 옵션과 1M PHY를 사용합니다.
 */
static const struct bt_le_adv_param ext_adv_param = {
    .id = BT_ID_DEFAULT,
    .sid = 0, /* Advertising Set ID */
    .secondary_max_skip = 0,
    // .options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M),
    .options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY),
    .interval_min = ADV_INT_MIN,
    .interval_max = ADV_INT_MAX,
    .peer = NULL,
};

static const uint8_t mfg_data_sr[] = {
    0xFF, 0xFF, // [12-13] Company ID (0xFFFF)
    0x01,       // [14] Data Structure Version
    0x01,       // [15] Registration Mode
    0x01,       // [16] Model Code
    0x01, 0x02  // [17-18] Firmware Version (v1.2, Major=1, Minor=2)
};

/** @brief 광고 세트 핸들 */
static struct bt_le_ext_adv *adv;

/** @brief 확장 광고를 시작합니다.
 *
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Ble_Start(void)
{
    int err = 0;

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err)
    {
        LOG_ERR("Failed to start extended advertising (err %d)", err);
        return err;
    }

    return err;
}

/** @brief 확장 광고를 중지합니다.
 *
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Ble_Stop(void)
{
    int err = 0;

    err = bt_le_ext_adv_stop(adv);
    if (err)
    {
        LOG_ERR("Failed to stop extended advertising (err %d)", err);
        return err;
    }

    return err;
}

/** @brief BLE 광고 모듈 초기화
 *
 * 블루투스 스택을 활성화하고 확장 광고 세트를 생성합니다.
 *
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Init_Ble(ble_init_t init_type)
{
    int err;

    switch (init_type)
    {
    case BLE_INIT:
        LOG_INF("Initializing Bluetooth...");
        err = bt_enable(NULL);
        if (err)
        {
            LOG_ERR("Bluetooth init failed (err %d)", err);
            return err;
        }
        break;

    case BLE_SCAN_RESPONSE:
        LOG_INF("BLE Scan Response Initialization...");
        err = bt_le_ext_adv_stop(adv);
        if (err)
        {
            LOG_ERR("Failed to stop adv (err %d)", err);
        }
        err = bt_le_ext_adv_delete(adv);
        if (err)
        {
            LOG_ERR("Failed to delete adv (err %d)", err);
        }

        // (EXT_ADV | SCANNABLE) 유지
        err = bt_le_ext_adv_create(BT_LE_ADV_PARAM(
                                       BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY,
                                       BT_GAP_ADV_FAST_INT_MIN_2,
                                       BT_GAP_ADV_FAST_INT_MAX_2,
                                       NULL),
                                   NULL, &adv);

        if (err)
        {
            LOG_ERR("Failed to create adv set (err %d)", err);
            return err;
        }
        /* 전송할 데이터 구조체 생성 */
        static struct bt_data sr[] = {
            BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
            BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_sr, sizeof(mfg_data_sr)),
        };

        // AD는 NULL/0, SD만 설정
        err = bt_le_ext_adv_set_data(adv,
                                     NULL, 0, // ★ AD 없음
                                     sr, ARRAY_SIZE(sr));
        if (err)
        {
            LOG_ERR("Failed to set Scan Response adv data (err %d)", err);
            return err;
        }

        err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_PARAM(SCAN_RSP_DURATION_MS, 0));
        if (err)
        {
            LOG_ERR("Failed to start Scan Response adv (err %d)", err);
            return err;
        }
        break;

    case BTN_ADV:
        LOG_INF("Creating advertising set...");

        err = bt_le_ext_adv_stop(adv);
        if (err)
        {
            LOG_ERR("Failed to stop adv (err %d)", err);
        }
        err = bt_le_ext_adv_delete(adv);
        if (err)
        {
            LOG_ERR("Failed to delete adv (err %d)", err);
        }

        /* adv 핸들에 광고 세트 생성 */
        err = bt_le_ext_adv_create(&ext_adv_param, NULL, &adv);
        if (err)
        {
            LOG_ERR("Failed to create advertising set (err %d)", err);
            return err;
        }
        break;

    default:
        break;
    }

    LOG_INF("BLE initialized. Device name: %s", CONFIG_BT_DEVICE_NAME);
    return 0;
}

/** @brief Manufacturer 데이터를 설정하고 1회성 광고를 수행합니다.
 *
 * 광고 데이터를 설정하고, Ble_Start() -> k_sleep() -> Ble_Stop()
 * 과정을 내부적으로 처리합니다.
 *
 * @param mfg Manufacturer Specific Data 페이로드 시작 주소.
 * @param mfg_len 페이로드 길이.
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Tx_Ble(const uint8_t *mfg, size_t mfg_len)
{
    int err = 0;

    /* 전송할 데이터 구조체 생성 */
    const struct bt_data ad =
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len);

    /* 1. 광고 데이터 설정 (ad 1개, scan response 없음) */
    err = bt_le_ext_adv_set_data(adv, &ad, 1, NULL, 0);
    if (err)
    {
        LOG_ERR("Failed to set advertising data (err %d)", err);
        return err;
    }

    /* 2. 광고 시작 */
    err = Ble_Start();
    if (err)
    {
        return err;
    }

    LOG_INF("Broadcasting for 200 ms...");
    k_sleep(ADV_ONE_SHOT_MS);

    /* 3. 광고 중지 */
    err = Ble_Stop();

    return err;
}