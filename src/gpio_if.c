/* gpio_if.c */
#include "gpio_if.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>


LOG_MODULE_REGISTER(gpio_if, LOG_LEVEL_INF);

//static struct nvs_fs fs;

/** @file gpio_if.c
 * @brief 보드 GPIO 인터페이스 구현 파일
 *
 * 버튼 인터럽트 처리, GPIO 초기 설정, LED/센서/RPU 제어 함수 등을 포함합니다.
 */

extern struct k_work_delayable loop_work;


/* ====== 버튼 인터럽트 판별 파라미터 ====== */
/** @brief 소프트웨어 디바운스 시간 (ms) */
#define BTN_DEBOUNCE_MS 20
/** @brief 짧게 눌린 것으로 판정하는 최대 시간 (ms) */
#define BTN_SHORT_MS 2000
/** @brief 길게 눌린 것으로 판정하는 최소 시간 (ms) */
#define BTN_LONG_MS  10000


/** @brief 보드의 모든 GPIO 핀에 대한 설정 구조체 */
static struct board_gpio g_cfg = {
    .led = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), led0_gpios, {0}),
    .btn = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), btn0_gpios, {0}),
    .io_int = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), io_int_gpios, {0}),
    .en_sensor = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), en_sensor_gpios, {0}),
    .en_rpu = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), en_rpu_gpios, {0}),
    .soh_alarm = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), soh_alarm_gpios, {0}),
    .soh_ok = GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), soh_ok_gpios, {0}),
};

/** @brief 현재 발생한 버튼 이벤트를 저장하는 아토믹 변수 */
static atomic_t g_btn_evt = ATOMIC_INIT(BTN_EVT_NONE);

/* 인터럽트 콜백/상태 */
/** @brief 버튼 GPIO 인터럽트 콜백 구조체 */
static struct gpio_callback g_btn_cb;
/** @brief 버튼 눌림이 시작된 시각 (k_uptime_get() 값), -1은 눌리지 않은 상태 */
static int64_t g_btn_pressed_at_ms = -1;
/** @brief 마지막으로 버튼 인터럽트가 발생한 시각 (디바운스용) */
static int64_t g_btn_last_irq_ms = 0;


/** @brief 버튼 엣지 인터럽트 콜백 (Active-Low)
 *
 * - Falling(1->0): 눌림 시작 시각 저장
 * - Rising (0->1): 떼는 순간 눌린 시간 계산 후 분류 출력
 *
 * @param dev  GPIO 포트 디바이스
 * @param cb   콜백 구조체
 * @param pins 핀 비트마스크
 */
static void btn_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    const int64_t now = k_uptime_get();

    /* 디바운스: 마지막 인터럽트로부터 최소 간격 보장 */
    if ((now - g_btn_last_irq_ms) < BTN_DEBOUNCE_MS)
    {
        return;
    }
    g_btn_last_irq_ms = now;

    /* Active-Low: level==0 이면 눌림 */
    int level = gpio_pin_get_dt(&g_cfg.btn);
    bool pressed = (level == 0);

    if (pressed)
		{
        g_btn_pressed_at_ms = now;				/* 눌림 시작 */
		}
    else
		{
		if (g_btn_pressed_at_ms >= 0)			/* 떼었을 때만 판정 */
			{
            int64_t held_ms = now - g_btn_pressed_at_ms;
			
			LOG_INF("Button: (%lld ms)\n", (long long)held_ms);

            if (held_ms >= 50 && held_ms <= 1000)
				{
				k_sleep(K_MSEC(300));
				NVIC_SystemReset ();
				}

			if (held_ms >= BTN_SHORT_MS && held_ms < BTN_LONG_MS)
				{
				atomic_set(&g_btn_evt, BTN_EVT_SHORT);
				}
			else 
				{
				if (held_ms >= BTN_LONG_MS)
					atomic_set(&g_btn_evt, BTN_EVT_LONG);
				else
					atomic_set(&g_btn_evt, BTN_EVT_NONE);
				}

			g_btn_pressed_at_ms = -1;

			k_work_cancel_delayable(&loop_work);		// 이전 슬립 시간 취소
			k_work_schedule(&loop_work, K_NO_WAIT);		// 바로 깨어남
			}
		}
}

/** @brief 발생한 버튼 이벤트 상태를 가져오고 상태를 초기화합니다.
 *
 * 이 함수는 아토믹 연산을 사용하여 버튼 이벤트를 **읽으면서 동시에 BTN_EVT_NONE으로 초기화**합니다.
 *
 * @return 마지막으로 발생한 버튼 이벤트 (btn_evt_t). 이벤트가 없으면 BTN_EVT_NONE.
 */
btn_evt_t Get_BtnStatus(void)
{
    /* atomic_set은 이전 값을 리턴하므로 ‘읽으면서 초기화’가 처리됨 */
    int prev = atomic_set(&g_btn_evt, BTN_EVT_NONE);
    return (btn_evt_t)prev;
}

/** @brief 보드의 모든 GPIO 핀들을 초기화하고 버튼 인터럽트를 설정합니다.
 *
 * GPIO 핀들의 출력/입력 방향과 초기 상태, 풀업/풀다운 저항을 설정합니다.
 *
 * @retval 0 성공
 * @retval -ENODEV GPIO 포트 디바이스를 찾을 수 없을 때
 * @retval 음수값 GPIO 설정 또는 인터럽트 설정 실패 시 Zephyr 에러 코드
 */
int board_gpio_init(void)
{
    const struct gpio_dt_spec *pins[] = {
        &g_cfg.led,
        &g_cfg.btn,
        &g_cfg.io_int,
        &g_cfg.en_sensor,
        &g_cfg.en_rpu,
        &g_cfg.soh_alarm,
        &g_cfg.soh_ok,
    };
    gpio_flags_t flags[] = {
        GPIO_OUTPUT_INACTIVE,      /**< led: 초기 비활성 출력 */
        GPIO_INPUT | GPIO_PULL_UP, /**< btn (AL): 풀업이 있는 입력 */
        GPIO_OUTPUT_ACTIVE,        /**< io_int: 초기 활성 출력 */
        GPIO_OUTPUT_ACTIVE,        /**< en_sensor: 초기 활성 출력 */
        GPIO_OUTPUT_ACTIVE,        /**< en_rpu: 초기 활성 출력 */
        GPIO_INPUT,                /**< soh_alarm: 입력 */
        GPIO_INPUT,                /**< soh_ok: 입력 */
    };

    for (int i = 0; i < ARRAY_SIZE(pins); ++i)
    {
        if (!device_is_ready(pins[i]->port))
            return -ENODEV;
        int r = gpio_pin_configure_dt(pins[i], flags[i]);
        if (r)
            return r;
    }

    /* 버튼 인터럽트: 양엣지(눌림/떼기) 설정 */
    int err = gpio_pin_interrupt_configure_dt(&g_cfg.btn, GPIO_INT_EDGE_BOTH);
    if (err)
    {
        printk("btn irq config failed: %d\n", err);
        return err;
    }

    /* 버튼 인터럽트 콜백 등록 */
    gpio_init_callback(&g_btn_cb, btn_gpio_cb, BIT(g_cfg.btn.pin));
    gpio_add_callback(g_cfg.btn.port, &g_btn_cb);


	pm_device_wakeup_enable(g_cfg.btn.port, true);

    return 0;
}

int board_led_set(bool on) { return gpio_pin_set_dt(&g_cfg.led, on); }

/** @brief 버튼 핀의 현재 상태를 가져옵니다.
 * @return 버튼이 눌렸으면 (Active-Low: 레벨 0) true, 아니면 false
 */
bool board_btn_get(void) { return !gpio_pin_get_dt(&g_cfg.btn); } /* active low */

/** @brief io_int 핀에 지정된 시간 동안 펄스(Low->High)를 발생시킵니다.
 * @param usec 펄스를 Low 상태로 유지하는 시간 (마이크로초)
 */
void io_int_pulse_us(uint32_t usec)
{
    gpio_pin_set_dt(&g_cfg.io_int, 0);
    k_busy_wait(usec);
    gpio_pin_set_dt(&g_cfg.io_int, 1);
}

/** @brief 센서 전원(en_sensor)을 제어합니다.
 * @param on true면 전원 켜짐(Active), false면 꺼짐(Inactive)
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int power_sensor(bool on) { return gpio_pin_set_dt(&g_cfg.en_sensor, on); }

/** @brief RPU 전원(en_rpu)을 제어합니다.
 * @param on true면 전원 켜짐(Active), false면 꺼짐(Inactive)
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int power_rpu(bool on) { return gpio_pin_set_dt(&g_cfg.en_rpu, on); }

/** @brief SOH_ALARM 핀의 상태를 가져옵니다.
 * @return 핀 레벨 상태 (0 또는 1)
 */
bool soh_alarm_get(void) { return gpio_pin_get_dt(&g_cfg.soh_alarm); }

/** @brief SOH_OK 핀의 상태를 가져옵니다.
 * @return 핀 레벨 상태 (0 또는 1)
 */
bool soh_ok_get(void) { return gpio_pin_get_dt(&g_cfg.soh_ok); }




extern struct flash_data cfg;

#define FLASH_OFFSET   0x7A000   // 기본 storage 영역
#define FLASH_SIZE     0x1000    // 4KB만 사용

static const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

//---------------------------------------------------------------------
int flash_init(void)
{
    if (!device_is_ready(flash_dev)) 
		{
        LOG_ERR("FLASH not ready");
        return -ENODEV;
		}
    return 0;
}
//----------------------------------------------------------------------------
int flash_write_data(void)
{
    int rc = flash_erase(flash_dev, FLASH_OFFSET, FLASH_SIZE);
    if (rc) {
        printk("flash erase failed: %d", rc);
        return rc;
    }

    cfg.index = FLASH_MAGIC;
    rc = flash_write(flash_dev, FLASH_OFFSET, &cfg, sizeof(cfg));
    if (rc) printk ("flash write failed: %d", rc);
	
   return rc;
}
//----------------------------------------------------------------------------
// 플래시에서 구조체 읽기
int flash_read_data(void)
{
    int rc = flash_read(flash_dev, FLASH_OFFSET, &cfg, sizeof(cfg));
    if (cfg.index != FLASH_MAGIC) 
		{
        printk("flash read failed: %d", rc);
		cfg.bat_value = 999;
		cfg.z = cfg.y = cfg.x = 0.21f;
		flash_write_data ();
		}

    printk ("\r\nFlash read OK     (Bat=%u (%.2f, %.2f, %.2f)\r\n",
            cfg.bat_value,  
			(double)cfg.x, (double)cfg.y, (double)cfg.z);
    return 0;
}

