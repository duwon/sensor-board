#include <zephyr/drivers/watchdog.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "wdt.h"

/* Sleep 최대 30초 + 여유 30초 = 60초 타임아웃 */
#define WDT_TIMEOUT_MS  35000

static const struct device *wdt_dev;
static int wdt_channel = -1;

void wdt_init(void)
{
    wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
    if (!device_is_ready(wdt_dev)) {
        printk("WDT not ready\n");
        return;
    }

    struct wdt_timeout_cfg cfg = {
        .flags      = WDT_FLAG_RESET_SOC,  /* 타임아웃 시 SOC 리셋 */
        .window.min = 0,
        .window.max = WDT_TIMEOUT_MS,
    };

    wdt_channel = wdt_install_timeout(wdt_dev, &cfg);
    if (wdt_channel < 0) {
        printk("WDT install failed: %d\n", wdt_channel);
        return;
    }

    /* 디버그 중 WDT 일시정지 */
    wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    printk("WDT started, timeout=%d sec\n", WDT_TIMEOUT_MS / 1000);
}

void wdt_feed_dog(void)
{
    if (wdt_dev && wdt_channel >= 0)
        wdt_feed(wdt_dev, wdt_channel);
}

/* 에러 등 강제 리셋이 필요할 때 호출 */
void wdt_reset_system(void)
{
    printk("WDT force reset!\n");
    k_sleep(K_MSEC(100));   /* printk flush 대기 */

    /* WDT feed 중단 → 타임아웃 대기 대신 즉시 리셋 */
    wdt_disable(wdt_dev);   /* 기존 WDT 중지 */

    /* 1ms 타임아웃으로 즉시 리셋 트리거 */
    struct wdt_timeout_cfg cfg = {
        .flags      = WDT_FLAG_RESET_SOC,
        .window.min = 0,
        .window.max = 1,    /* 1ms → 즉시 리셋 */
    };
    wdt_install_timeout(wdt_dev, &cfg);
    wdt_setup(wdt_dev, 0);
    /* feed 안하면 1ms 후 리셋 */
    while (1) {}
}