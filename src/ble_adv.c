
#include "ble_adv.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <string.h>

static struct bt_le_ext_adv *adv;

static int build_new_packet(uint8_t *mfg, size_t *mfg_len, const sensor_sample_t *smp){
    /* Manufacturer specific payload only; flags will be a separate AD */
    uint8_t p[40] = {0};
    int idx = 0;
    /* Company ID (test): 0xFFFF */
    p[idx++] = 0xFF; p[idx++] = 0xFF;
    /* Structure ver */
    p[idx++] = 0x01;
    /* Model code example */
    p[idx++] = 0x01;
    /* Device Status / Error / MCU Temp */
    p[idx++] = 0x00;
    p[idx++] = 0x00;
    p[idx++] = 25; /* dummy mcu temp */
    /* Battery */
    p[idx++] = smp->battery_pc;
    /* Value Presence Mask */
    p[idx++] = 0x01;
    int32_t v[6] = {smp->p_value_x100, 0,0,0,0,0};
    for (int i=0;i<6;i++){
        p[idx++] = (uint8_t)(v[i] & 0xFF);
        p[idx++] = (uint8_t)((v[i] >> 8) & 0xFF);
        p[idx++] = (uint8_t)((v[i] >> 16) & 0xFF);
        p[idx++] = (uint8_t)((v[i] >> 24) & 0xFF);
    }
    memcpy(mfg, p, idx);
    *mfg_len = idx;
    return 0;
}

int ble_init(void){
    int err = bt_enable(NULL);
    if (err) return err;

    struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_EXT_ADV, /* extended */
        160, 160, /* interval */
        NULL
    );
    err = bt_le_ext_adv_create(&param, NULL, &adv);
    return err;
}

int ble_update_and_advertise(const struct dip_bits *dip, const sensor_sample_t *smp){
    uint8_t mfg[40]; size_t mfg_len = 0;
    build_new_packet(mfg, &mfg_len, smp);

    /* Flags AD (general discoverable + BR/EDR not supported) */
    const uint8_t flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
    struct bt_data ad[2] = {
        BT_DATA(BT_DATA_FLAGS, &flags, sizeof(flags)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len),
    };

    int err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) return err;
    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    return err;
}
