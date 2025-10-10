
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "gpio_if.h"
#include "dip_switch.h"
#include "ltc3337.h"
#include "sensors.h"
#include "ble_adv.h"

static struct k_work_delayable loop_work;
static struct dip_bits g_dip;

/* --- LED heartbeat --- */
static struct k_work_delayable led_work;
static uint32_t led_period_ms = 500; /* 정상: 0.5s 토글 */
static bool led_state = false;

static void led_fn(struct k_work *w)
{
	led_state = !led_state;
	board_led_set(led_state); /* P0.11 Active High */
	k_work_schedule(&led_work, K_MSEC(led_period_ms));
}
/* --- end of LED heartbeat --- */

static void loop_fn(struct k_work *w)
{
	/* read sensors */
	sensor_sample_t s = {0};
	read_pressure_0x28(&s);
	int16_t t_cx100 = 0;
	read_ntc_ain1_cx100(&t_cx100);
	s.temperature_c_x100 = t_cx100;
	s.battery_pc = 100; /* TODO: via LTC3337 or ADC */

	/* BLE advertise */
	int err = ble_update_and_advertise(&g_dip, &s);

	/* LED 패턴: 성공=0.5s, 실패=0.12s 고속 깜빡임 */
	led_period_ms = (err == 0) ? 500 : 120;

	/* schedule next based on DIP period bit: H=Long(10s), L=Short(5s) */
	uint32_t next_ms = g_dip.period ? 10000 : 5000;
	k_work_schedule(&loop_work, K_MSEC(next_ms));
}

void main(void)
{
	printk("Sensor Board FW boot\n");
	board_gpio_init();

	uint8_t raw = 0;
	dip_read_u8(&raw);
	g_dip = parse_dip(raw);

	sensors_init();
	ltc3337_init();
	ble_init();

	/* LED heartbeat start */
	k_work_init_delayable(&led_work, led_fn);
	k_work_schedule(&led_work, K_MSEC(200));

	k_work_init_delayable(&loop_work, loop_fn);
	k_work_schedule(&loop_work, K_SECONDS(1));
}
