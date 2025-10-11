#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "gpio_if.h"
#include "dip_switch.h"
#include "ltc3337.h"
#include "sensors.h"
#include "ble_adv.h"
#include "sleep_if.h"
#include "debug.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static atomic_t g_diag_log_on = ATOMIC_INIT(1);
void app_diag_log_enable(bool en){ atomic_set(&g_diag_log_on, en); }
bool app_diag_log_is_enabled(void){ return atomic_get(&g_diag_log_on); }

static struct k_work_delayable loop_work;
static struct dip_bits g_dip;

static struct k_work_delayable led_work;
static uint32_t led_period_ms = 500;
static bool led_state;
static void led_fn(struct k_work *w){
    led_state = !led_state;
    board_led_set(led_state);
    k_work_schedule(&led_work, K_MSEC(led_period_ms));
}

/* DIP → BLE 설정 반영 */
static void apply_ble_cfg_from_dip(const struct dip_bits *d)
{
    ble_cfg_t c = {
        .legacy      = d->legacy ? true : false,          /* 1=Legacy(요구안), 0=EXT */
        .interval_ms = d->period ? 10000 : 5000,          /* 10s / 5s */
        .phy         = d->phy ? PHY_1M : PHY_CODED,       /* 1M / Coded */
        .connectable = false,
    };
    (void)Init_Ble(&c);
    LOG_INF("BLE cfg applied: legacy=%u, phy=%s, interval=%ums",
            c.legacy, (c.phy==PHY_CODED?"coded":"1M"), c.interval_ms);
}

static void loop_fn(struct k_work *w)
{
    /* 1) DIP 재확인 */
    uint8_t raw=0; dip_read_u8(&raw);
    g_dip = parse_dip(raw);

    /* 2) (임시) 센서 샘플 구성 */
    sensor_sample_t s = {0};
    read_pressure_0x28(&s);
    int16_t t_x100=0; read_ntc_ain1_cx100(&t_x100);
    s.temperature_c_x100 = t_x100;
    s.battery_pc = 100;

    /* 3) 광고용 Manufacturer payload 만들기 (간단 20B 내) */
    uint8_t mfg[24]; size_t mfg_len=0;
    {
        int32_t v1 = s.p_value_x100;
        uint8_t p[24]; int idx=0;
        p[idx++]=0xFF; p[idx++]=0xFF; /* CID test */
        p[idx++]=0x01; p[idx++]=0x01; /* ver, model */
        p[idx++]=0x00; p[idx++]=0x00; /* status, err */
        p[idx++]=25;                  /* mcuT dummy */
        p[idx++]=s.battery_pc;        /* batt */
        p[idx++]=0x01;                /* presence: v1 */
        p[idx++]=(uint8_t)(v1);
        p[idx++]=(uint8_t)(v1>>8);
        p[idx++]=(uint8_t)(v1>>16);
        p[idx++]=(uint8_t)(v1>>24);
        mfg_len = MIN(sizeof(mfg),(size_t)idx);
        memcpy(mfg,p,mfg_len);
    }

    /* 4) Scan Response 구성 → Rx_Ble() (스캔요청 응답) */
    /* 예: 짧은 이름/간단 상태를 scan response로 실음 */
    char sr[20];
    int sr_len = snprintk(sr, sizeof(sr), "ARX-T %u%%", s.battery_pc);
    if (sr_len < 0) sr_len = 0;
    Rx_Ble((const uint8_t*)sr, (size_t)sr_len);

    /* 5) 브로드캐스트 시작/갱신 (짧게 노출) */
    int err = Tx_Ble(mfg, mfg_len);
    bool ok = (err==0)||(err==-EALREADY)||(err==-EINPROGRESS);
    led_period_ms = ok ? 500 : 120;

    /* 6) 광고 노출 윈도우 (예: 200ms) 후 정지 */
    k_sleep(K_MSEC(200));
    Ble_Stop();

    /* 7) 슬립 길이 = DIP [5] (10s/5s) → 슬립 & 웨이크 */
    uint32_t sleep_ms = g_dip.period ? 10000 : 5000;
    if (app_diag_log_is_enabled()) {
        LOG_INF("loop: sleep=%ums, err=%d, P=%ld x100, T=%d x100, batt=%u%%, DIP{legacy=%u phy=%u period=%u}",
                sleep_ms, err, (long)s.p_value_x100, s.temperature_c_x100, s.battery_pc,
                g_dip.legacy, g_dip.phy, g_dip.period);
    }
    Start_Sleep(sleep_ms/1000);
    Wakeup(); /* 깨어난 후 전원/IF 재가동 → 다음 루프에서 센서수집(여기선 간단 샘플 유지) */

    /* 8) 다음 루프 즉시 실행 (깨어난 뒤 바로 광고/슬립 반복) */
    k_work_schedule(&loop_work, K_MSEC(1));
}

void main(void)
{
    printk("Sensor Board FW boot\n");
    board_gpio_init();

    uint8_t raw=0; dip_read_u8(&raw);
    g_dip = parse_dip(raw);

    sensors_init();
    ltc3337_init();
    ble_init();
    apply_ble_cfg_from_dip(&g_dip);

    debug_run_startup();

    k_work_init_delayable(&led_work, led_fn);
    k_work_schedule(&led_work, K_MSEC(200));

    k_work_init_delayable(&loop_work, loop_fn);
    k_work_schedule(&loop_work, K_SECONDS(1));
}
