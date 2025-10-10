#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_echo, LOG_LEVEL_INF);

/* 콘솔로 선택된 UART 장치 사용(overlay/Kconfig에서 콘솔=UART0로 설정돼 있음) */
#define UART_NODE DT_CHOSEN(zephyr_console)
const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

static void uart_echo_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    if (!device_is_ready(uart_dev)) {
        /* 장치가 준비 안 됨 */
        for (;;) { k_msleep(1000); }
    }

    /* 안내 메시지 (printk 대신 TX 드라이버로 출력) */
    const char *banner = "\r\n[UART ECHO] Type something (CR or CR+LF to newline)\r\n";
    for (const char *p = banner; *p; ++p) {
        uart_poll_out(uart_dev, *p);
    }

    for (;;) {
        unsigned char ch;
        /* 문자가 들어오면 0 리턴, 없으면 -1 */
        if (uart_poll_in(uart_dev, &ch) == 0) {
            /* 그대로 에코 */
            uart_poll_out(uart_dev, ch);
            /* 보기 좋게 개행 정리 */
            if (ch == '\r') uart_poll_out(uart_dev, '\n');
        } else {
            /* 바쁜 루프 방지 */
            k_sleep(K_MSEC(1));
        }
    }
}

/* 스레드 생성 */
K_THREAD_STACK_DEFINE(uart_echo_stack, 1024);
static struct k_thread uart_echo_tid;
static int uart_echo_init(void)
{
    k_thread_create(&uart_echo_tid, uart_echo_stack, K_THREAD_STACK_SIZEOF(uart_echo_stack),
                    uart_echo_thread, NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);
    k_thread_name_set(&uart_echo_tid, "uart_echo");
    return 0;
}
SYS_INIT(uart_echo_init, APPLICATION, 50);
