#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "gpio_if.h"
#include "dip_switch.h"
#include "ltc3337.h"
#include "sensors.h"
#include "ble_adv.h"
#include "debug.h"
#include "app_diag.h"
#include "sleep_if.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* 진단 로그 스위치 */
static atomic_t g_diag_log_on = ATOMIC_INIT(1);
static inline bool diag_on(void) { return atomic_get(&g_diag_log_on); }

/* 주기 작업 */
static struct k_work_delayable loop_work;
static struct dip_bits g_dip;

/* LED heartbeat */
static struct k_work_delayable led_work;
static uint32_t led_period_ms = 500;
static bool led_state = false;
static void led_fn(struct k_work *w)
{
	led_state = !led_state;
	board_led_set(led_state);
	k_work_schedule(&led_work, K_MSEC(led_period_ms));
}

/* Manufacturer payload 생성 */
static size_t build_basic_mfg(uint8_t *out, size_t cap,
							  int32_t p_x100, int16_t t_cx100, uint8_t batt_pc)
{
	/* Company ID 임시 0xFFFF + 버전 등 최소 필드 */
	if (cap < 12)
		return 0;
	size_t idx = 0;
	out[idx++] = 0xFF;
	out[idx++] = 0xFF;			   /* Company ID (test) */
	out[idx++] = 0x01;			   /* Struct ver */
	out[idx++] = 0x01;			   /* Model */
	out[idx++] = (uint8_t)batt_pc; /* battery(%) */
	/* pressure 4B LE */
	out[idx++] = (uint8_t)(p_x100);
	out[idx++] = (uint8_t)(p_x100 >> 8);
	out[idx++] = (uint8_t)(p_x100 >> 16);
	out[idx++] = (uint8_t)(p_x100 >> 24);
	/* temp 2B LE */
	out[idx++] = (uint8_t)(t_cx100);
	out[idx++] = (uint8_t)(t_cx100 >> 8);
	return idx;
}

static void loop_fn(struct k_work *w)
{
	/* 1) 센서 읽기(간단) */
	sensor_sample_t s = {0};
	read_pressure_0x28(&s);
	int16_t t_cx100 = 0;
	read_ntc_ain1_cx100(&t_cx100);
	s.temperature_c_x100 = t_cx100;
	s.battery_pc = 100;

	/* 2) 기본 Manufacturer 패킷 빌드 */
	uint8_t mfg[24];
	size_t mfg_len = build_basic_mfg(mfg, sizeof(mfg),
									 s.p_value_x100, s.temperature_c_x100, s.battery_pc);

	/* 3) 광고 시작/갱신 (브로드캐스트 전용) */
	int err = Tx_Ble(mfg, mfg_len);

	/* 4) LED 속도(정상/에러) */
	bool ok = (err == 0) || (err == -EALREADY) || (err == -EINPROGRESS);
	led_period_ms = ok ? 500 : 120;

	/* 5) 다음 주기 (DIP: 1=10s, 0=5s) */
	uint32_t next_ms = g_dip.period ? 10000 : 5000;

	if (diag_on())
	{
		LOG_INF("loop: sleep=%ums, err=%d, P=%ld x100, T=%d x100, batt=%u%%, DIP{legacy-only, period=%u}",
				next_ms, err, (long)s.p_value_x100, s.temperature_c_x100, s.battery_pc, g_dip.period);
	}

	/* 필요 시 여기서 Sleep/Wakeup 시퀀스 삽입 */
	Start_Sleep(next_ms / 1000); // 호출 위치/타이밍 결정
	Wakeup();

	k_work_schedule(&loop_work, K_MSEC(next_ms));
}

int main(void)
{
	printk("Sensor Board FW boot\n");
	board_gpio_init();

	uint8_t raw = 0;
	dip_read_u8(&raw);
	g_dip = parse_dip(raw);

	sensors_init();
	ltc3337_init();

	/* BLE 초기화 + 인터벌 설정 (레거시 비연결 전용) */
	Init_Ble();

	/* 디버깅 코드 */
	debug_run_startup();

	/* LED HB */
	k_work_init_delayable(&led_work, led_fn);
	k_work_schedule(&led_work, K_MSEC(200));

	k_work_init_delayable(&loop_work, loop_fn);
	k_work_schedule(&loop_work, K_SECONDS(1));

	return 0;
}
