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

static uint8_t mfg_data_sr[] = {
    0xFF, 0xFF, // [12-13] Company ID (0xFFFF)
    0x01,       // [14] Data Structure Version
    0x01,       // [15] Registration Mode
    0x01,       // [16] Model Code
    0x01, 0x02  // [17-18] Firmware Version (v1.2, Major=1, Minor=2)
};


static char scan_name[10];
static struct bt_data sr[] = {
    {   // 이름은 나중에 채움
        .type = BT_DATA_NAME_COMPLETE,
        .data = scan_name,
        .data_len = 9,
    },
    {   // 제조사 데이터는 그대로
        .type = BT_DATA_MANUFACTURER_DATA,
        .data = mfg_data_sr,
        .data_len = sizeof(mfg_data_sr),
    },
};

static struct bt_le_ext_adv *adv;
extern struct Status Stat;
//--------------------------------------------------------------------
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
//--------------------------------------------------------------------
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
//-------------------------------------------------------------------------
void Set_Device_Name()
{
	switch (Stat.Dipsw & 0x0f)
		{
		case 1  :										// 속도 16g
			snprintk(scan_name, sizeof(scan_name), "ARX-AT446"); break;
		case 9  :										// 속도 4g
			snprintk(scan_name, sizeof(scan_name), "ARX-AT445"); break;
		case 2  :										// 가속도 16g
			snprintk(scan_name, sizeof(scan_name), "ARX-AT436"); break;
		case 10 :										// 가속도 4g
			snprintk(scan_name, sizeof(scan_name), "ARX-AT435"); break;
		case 3	:										// ntc
		case 11	:
			snprintk(scan_name, sizeof(scan_name), "ARX-AT205"); break;
		case 4  :										// air flow SSCDJNN002ND2A3 59,8 mmh2o		0x28
			snprintk(scan_name, sizeof(scan_name), "ARX-AT186"); break;
		case 12 :										// air flow XGZP6897D001KPDPN 100mmh2o		0x58
			snprintk(scan_name, sizeof(scan_name), "ARX-AT185"); break;
			break;
		case 5  :										// air header SSCDJNN010BA2A3 0~10 bar		0x28
			snprintk(scan_name, sizeof(scan_name), "ARX-AT146"); break;
		case 13 :										// air header XGZP6847DC001MPGPN -1~10bar 	0x6d
			snprintk(scan_name, sizeof(scan_name), "ARX-AT145"); break;
		case 6  :										// outlet SSCDJNN100MD2A3(1020h2o)			0x28
			snprintk(scan_name, sizeof(scan_name), "ARX-AT126"); break;
		case 14 :										// outlet XGZP6897D010KPDPN(1000h2o)		0x58
			snprintk(scan_name, sizeof(scan_name), "ARX-AT125"); break;
		case 7  :										// inlet  SSCDJNN100MD2A3(1020h2o)			0x28
			snprintk(scan_name, sizeof(scan_name), "ARX-AT116"); break;
		case 15 :										// inlet  XGZP6897D010KPDPN(1000h2o)		0x58
			snprintk(scan_name, sizeof(scan_name), "ARX-AT115"); break;
		}	
	
	sr[0].data = scan_name;
	sr[0].data_len = 9;

}
//-------------------------------------------------------------------------
int Init_Ble(ble_init_t init_type)
{
int err;

    switch (init_type)
    {
    case BLE_SCAN_RESPONSE:
	
		Set_Device_Name ();
	
        LOG_INF("BLE Scan Response Initialization...");
        err = bt_le_ext_adv_stop(adv);
        if (err) LOG_ERR("Failed to stop adv (err %d)", err);

        err = bt_le_ext_adv_delete(adv);
        if (err) LOG_ERR("Failed to delete adv (err %d)", err);

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

        mfg_data_sr[4] = Stat.Model;
		
		err = bt_le_ext_adv_set_data(adv, NULL, 0, sr, ARRAY_SIZE(sr));  // ad 없음
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

/*
//-------------------------------------------------------------------------
int Init_Ble(ble_init_t init_type)
{
static struct bt_data sr[] = 
		{
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_sr, sizeof(mfg_data_sr)),
		};


    switch (init_type)
    {
    case BLE_SCAN_RESPONSE:
        LOG_INF("BLE Scan Response Initialization...");
        err = bt_le_ext_adv_stop(adv);
        if (err) LOG_ERR("Failed to stop adv (err %d)", err);

        err = bt_le_ext_adv_delete(adv);
        if (err) LOG_ERR("Failed to delete adv (err %d)", err);

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

        mfg_data_sr[4] = Stat.Model;
		
		err = bt_le_ext_adv_set_data(adv, NULL, 0, sr, ARRAY_SIZE(sr));  // ad 없음
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
*/
//----------------------------------------------------------------------------
int Tx_Ble(const uint8_t *mfg, size_t mfg_len)
{
    int err = 0;
    const struct bt_data ad =
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, mfg_len);

    err = bt_le_ext_adv_set_data(adv, &ad, 1, NULL, 0);
    if (err)
		{
        LOG_ERR("Failed to set advertising data (err %d)", err);
        return err;
		}

    err = Ble_Start();
    if (err)  return err;

    LOG_INF("Broadcasting for 200 ms...");
    k_sleep(ADV_ONE_SHOT_MS);

    err = Ble_Stop();

    return err;
}
//--------------------------------------------------------------
void ble_setup (uint8_t phy, uint8_t scan)
{
int err;
    const struct bt_data *sd_ptr;
    size_t sd_len;
    struct bt_le_adv_param param = {
        .id               = BT_ID_DEFAULT,
        .sid              = 0,
        .secondary_max_skip = 0,
        .options          = BT_LE_ADV_OPT_EXT_ADV |
                            BT_LE_ADV_OPT_CONNECTABLE |
                            BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min     = (phy == 1) ?
                              ADV_INT_MIN :
                              BT_GAP_ADV_SLOW_INT_MIN,
        .interval_max     = (phy == 1) ?
                              ADV_INT_MIN :
                              BT_GAP_ADV_SLOW_INT_MAX,
        .peer             = NULL,
    };


    if (!adv) {
        printk("adv is NULL, call ble_adv_ext_init() first\n");
        return;
    }

    printk("ADV start: PHY=%s, MODE=%s\n",
           (phy  == 0) ? "Code8" : "1M",
           (scan == 0) ? "ADV only" : "ADV+SCAN_RSP");

    bt_le_ext_adv_stop(adv);

	if (scan) phy = 1;							// code8 이면 phy 를 1M 로 변경

	if (phy) 
		param.options &= ~BT_LE_ADV_OPT_CODED;
	else
		param.options |= BT_LE_ADV_OPT_CODED;
	
	if (scan)  
		{
		param.options |= BT_LE_ADV_OPT_SCANNABLE;
		param.options &= ~BT_LE_ADV_OPT_CONNECTABLE;
		Init_Ble(BLE_SCAN_RESPONSE);
		} 
	else 
		{
        param.options &= ~BT_LE_ADV_OPT_SCANNABLE;
        param.options &= ~BT_LE_ADV_OPT_CONNECTABLE; // ADV only면
		Init_Ble(BTN_ADV);
		}

    err = bt_le_ext_adv_update_param(adv, &param);
    if (err) {
        printk("bt_le_ext_adv_update_param err %d\n", err);
        return;
    }
}	
		
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "NODE-ADV", 8),
};	
//------------------------------------------------------------------
int ble_adv_ext_init(void)
{
 int err;

    /* 기본은 1M용 설정 (options로만 제어) */
    struct bt_le_adv_param adv_param = {
        .id               = BT_ID_DEFAULT,
        .sid              = 0,
        .secondary_max_skip = 0,
        .options          = BT_LE_ADV_OPT_EXT_ADV |
                            BT_LE_ADV_OPT_CONNECTABLE |
                            BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min     = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max     = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer             = NULL,
    };

    err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
    if (err) {
        printk("bt_le_ext_adv_create failed (err %d)\n", err);
        return err;
    }

    /* 초기값은 대충 넣어두고, 실제로는 ble_exe에서 다시 세팅 */
    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("bt_le_ext_adv_set_data failed (err %d)\n", err);
        return err;
    }

    printk("Extended ADV init OK (non-connectable)\n");
    return 0;
}



