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
#define ADV_INT_MIN_100MS 0x00A0
/** @brief 최대 광고 인터벌 (100ms = 0x00A0 * 0.625ms) */
#define ADV_INT_MAX_100MS 0x00A0

/** @brief 확장 광고 파라미터 구조체
 *
 * 비연결(Non-connectable), 비검색(Non-scannable) 옵션과 1M PHY를 사용합니다.
 */
static const struct bt_le_adv_param ext_adv_param = {
    .id = BT_ID_DEFAULT,
    .sid = 0, /* Advertising Set ID */
    .secondary_max_skip = 0,
    .options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY |
                BT_LE_ADV_OPT_USE_TX_POWER | BT_LE_ADV_OPT_NO_2M),
    .interval_min = ADV_INT_MIN_100MS,
    .interval_max = ADV_INT_MAX_100MS,
    .peer = NULL,
};

/** @brief 광고 세트 핸들 (모듈 내에서 공유) */
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
int Init_Ble(void)
{
    int err;

    LOG_INF("Initializing Bluetooth...");
    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Creating advertising set...");
    /* adv 핸들에 광고 세트 생성 */
    err = bt_le_ext_adv_create(&ext_adv_param, NULL, &adv);
    if (err)
    {
        LOG_ERR("Failed to create advertising set (err %d)", err);
        return err;
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