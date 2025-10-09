/* main.c - Legacy ADV (1M) + Manufacturer Data + 10s update
 * 기존 beacon 예제 흐름을 최대한 유지: ad[]/sd[] 배열 + bt_le_adv_start()
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* ---- 제조사 데이터 버퍼 (legacy ADV는 총 31B 제한!!!) ----
 * 여기서는 예제로 20B 내외만 사용. 실제 '신형 38B'는 확장광고에서만 가능함.
 * 필요하면 Scan Response(sd[])에 몇 바이트 더 분산해 실험해볼 수 있음.
 */
static uint8_t mfg_payload[20];  /* 가볍게 시작: 스캐너 호환성 우선 */

static struct k_work_delayable adv_work;

/* AD(광고) / SD(스캔응답) — 예제 원형 유지 */
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    /* 제조사 데이터(0xFF). Company ID는 임시 0xFFFF → 실제 코드로 교체 */
    /* data 포인터는 런타임에 mfg_payload를 채우고 가리키게 함 */
    { .type = BT_DATA_MANUFACTURER_DATA, .data = mfg_payload, .data_len = sizeof(mfg_payload) },
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* 제조사 데이터 구성(예시) */
static void build_mfg_payload(void)
{
    /* 예시 포맷:
       [0..1] CompanyID(LE) = 0xFFFF
       [2]    StructVer     = 0x01
       [3]    Model         = 0x01
       [4]    Battery%      = 0x64
       [5]    PresenceMask  = 0x2F
       [6..]  센서값(간단 예: 온도 x100, int16)
    */
    memset(mfg_payload, 0, sizeof(mfg_payload));
    mfg_payload[0] = 0xFF;  /* CompanyID LSB */
    mfg_payload[1] = 0xFF;  /* CompanyID MSB */
    mfg_payload[2] = 0x01;  /* StructVer */
    mfg_payload[3] = 0x01;  /* Model */
    mfg_payload[4] = 0x64;  /* Battery 100% */
    mfg_payload[5] = 0x2F;  /* PresenceMask */

    /* 센서 예시: 25.34°C * 100 = 2534 -> int16 LE */
    int16_t t_x100 = 2534;
    mfg_payload[6] = (uint8_t)(t_x100 & 0xFF);
    mfg_payload[7] = (uint8_t)((t_x100 >> 8) & 0xFF);

    /* 나머지는 0으로 유지(필요 시 확장) */
}

/* 10초마다 광고 데이터 업데이트 (예제 원형 유지: bt_le_adv_update_data 사용) */
static void adv_update_work(struct k_work *work)
{
    ARG_UNUSED(work);

    build_mfg_payload();

    /* ad[1]의 data_len이 바뀔 수 있으니(여기선 고정) 안전하게 재지정 */
    ad[1].data = mfg_payload;
    ad[1].data_len = sizeof(mfg_payload);

    int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("bt_le_adv_update_data err %d\n", err);
    } else {
        printk("ADV updated (%uB mfg)\n", (unsigned)ad[1].data_len);
    }

    k_work_reschedule(&adv_work, K_SECONDS(10));
}

static void bt_ready(int err)
{
    char addr_s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t addr = {0};
    size_t count = 1;

    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    build_mfg_payload();

    /* 예제 원형 유지: legacy 1M, 스캐너 호환성 매우 좋음 */
    err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    /* 주기 갱신 타이머 시작 */
    k_work_init_delayable(&adv_work, adv_update_work);
    k_work_schedule(&adv_work, K_SECONDS(10));

    bt_id_get(&addr, &count);
    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));
    printk("Beacon started, advertising as %s\n", addr_s);
}

int main(void)
{
    printk("Starting Beacon (legacy ADV + MFG + periodic update)\n");

    int err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }
    return 0;
}
