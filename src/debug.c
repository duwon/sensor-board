#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "debug.h"
#include "app_diag.h"
#include "dip_switch.h" /* Get_Switch() */
#include "ble_adv.h"
#include "sensors.h"
#include "gpio_if.h"
#include "lsm6dso.h"
#include "lsm6dso.h"

LOG_MODULE_REGISTER(app_dbg, LOG_LEVEL_INF);

/* I2C0 핸들 */
static const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

#if IS_ENABLED(CONFIG_SHELL)
static int cmd_echo(const struct shell *sh, size_t argc, char **argv);
static int cmd_i2c_scan(const struct shell *sh, size_t argc, char **argv);
static int cmd_imu_who(const struct shell *sh, size_t argc, char **argv);
static int cmd_dip_read(const struct shell *sh, size_t argc, char **argv);
static int cmd_log_sw(const struct shell *sh, size_t argc, char **argv);
#endif

/* ─────────────────────────────────────────────
 * I2C 전체 스캔 (0x03~0x77)
 *  - 트랜잭션 명확히 보이도록 write_read 사용 (0바이트 write 회피)
 * ────────────────────────────────────────────*/
int i2c_bus_scan(void)
{
    if (!device_is_ready(i2c0_dev))
    {
        LOG_ERR("i2c0 not ready");
        return -ENODEV;
    }
    LOG_INF("I2C0 scan start");

    uint8_t reg = 0x00;
    uint8_t val = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++)
    {
        /* 레지스터 없는 디바이스도 있어 읽기만 시도 → 많은 경우 NACK 허용 */
        int r = i2c_write_read(i2c0_dev, addr, &reg, 1, &val, 1);
        if (r == 0)
        {
            LOG_INF("I2C found: 0x%02X", addr);
        }
        k_busy_wait(50);
    }
    LOG_INF("I2C0 scan done");
    return 0;
}

/* ─────────────────────────────────────────────
 * LSM6DSO WHO_AM_I 확인 (0x6A)
 * ────────────────────────────────────────────*/
#define LSM6DSO_ADDR 0x6A
#define WHO_AM_I 0x0F
#define LSM6DSO_ID 0x6C

int lsm6dso_probe(void)
{
    if (!device_is_ready(i2c0_dev))
        return -ENODEV;

    uint8_t reg = WHO_AM_I, id = 0;
    int r = i2c_write_read(i2c0_dev, LSM6DSO_ADDR, &reg, 1, &id, 1);
    if (r)
    {
        LOG_ERR("LSM6DSO WHO_AM_I read err: %d", r);
        return r;
    }
    LOG_INF("LSM6DSO WHO_AM_I=0x%02X (%s)", id, (id == LSM6DSO_ID) ? "OK" : "UNEXPECTED");
    return (id == LSM6DSO_ID) ? 0 : -EIO;
}

/* ─────────────────────────────────────────────
 * 부팅 직후 한 번 호출해 사용하는 디버그 러너
 * ────────────────────────────────────────────*/
void debug_run_startup(void)
{
    /* I2C 디바그 */
    i2c_bus_scan();

    /* LSM6DSO WHO_AM_I */
    lsm6dso_probe();
}

/* ─────────────────────────────────────────────
 * ====================== 쉘 명령 구현 ==========
 * ────────────────────────────────────────────*/
#if IS_ENABLED(CONFIG_SHELL)

static int cmd_echo(const struct shell *sh, size_t argc, char **argv)
{
    /* RX 검증용: 입력받은 문자열 그대로 출력 */
    for (size_t i = 1; i < argc; i++)
    {
        shell_fprintf(sh, SHELL_NORMAL, "%s%s", argv[i], (i + 1 < argc) ? " " : "");
    }
    shell_fprintf(sh, SHELL_NORMAL, "\n");
    return 0;
}

static int cmd_i2c_scan(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    int r = i2c_bus_scan();
    shell_print(sh, "i2c_scan ret=%d", r);
    return r;
}

static int cmd_imu_who(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    int r = lsm6dso_probe();
    shell_print(sh, "lsm6dso ret=%d", r);
    return r;
}

static int cmd_dip_read(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    uint8_t raw = 0;
    struct dip_bits d = {0};
    extern int Get_Switch(uint8_t *raw, struct dip_bits *parsed);
    int r = Get_Switch(&raw, &d);
    if (r == 0)
    {
        shell_print(sh, "DIP RAW=0x%02X model=%u mode=%u phy=%u period=%u pwr=%u legacy=%u",
                    raw, d.model, d.mode_sub, d.phy, d.period, d.pwr, d.legacy);
    }
    else
    {
        shell_error(sh, "Get_Switch fail: %d", r);
    }
    return r;
}

static int cmd_log_sw(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1)
    {
        shell_print(sh, "diag log is %s", app_diag_log_is_enabled() ? "ON" : "OFF");
        return 0;
    }
    if (strcmp(argv[1], "on") == 0)
    {
        app_diag_log_enable(true);
        shell_print(sh, "diag log ON");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0)
    {
        app_diag_log_enable(false);
        shell_print(sh, "diag log OFF");
        return 0;
    }
    shell_error(sh, "usage: diag log [on|off]");
    return -EINVAL;
}

/* --- NTC 온도 읽기 커맨드 ---------------------------------------- */
static int cmd_ntc(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int16_t vdd_mv = 0;
    int16_t cx100 = 0;

    int rc = read_vdd_mv(&vdd_mv);
    if (rc)
    {
        shell_error(sh, "VDD read failed: %d", rc);
        return rc;
    }
    rc = read_ntc_ain1_cx100(&cx100);
    if (rc)
    {
        shell_error(sh, "NTC read failed: %d", rc);
        return rc;
    }

    int16_t abs_cx = (cx100 < 0) ? -cx100 : cx100;
    shell_print(sh, "VDD=%d mV, NTC=%s%d.%02d °C",
                vdd_mv,
                (cx100 < 0) ? "-" : "",
                abs_cx / 100, abs_cx % 100);
    return 0;
}

/* --- BLE 제어 커맨드 ---------------------------------------- */
static int cmd_ble_start(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* 홀드 해제 먼저 */
    app_set_hold(false);

    int rc = Ble_Start();
    if (rc == 0)
        shell_print(shell, "BLE advertising started; loop resumed.");
    else
        shell_error(shell, "BLE start failed: %d", rc);
    return rc;
}

static int cmd_ble_stop(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    int rc = Ble_Stop();
    app_set_hold(true);

    if (rc == 0)
        shell_print(shell, "BLE advertising stopped.");
    else
        shell_error(shell, "BLE stop failed: %d", rc);
    return rc;
}

/* diag ble ... 트리 구성 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_ble,
                               SHELL_CMD(start, NULL, "Start BLE advertising (legacy/ext auto).", cmd_ble_start),
                               SHELL_CMD(stop, NULL, "Stop BLE advertising.", cmd_ble_stop),
                               SHELL_SUBCMD_SET_END);

/* --- GPIO 제어 커맨드 ---------------------------------------- */
static int do_set(const struct shell *shell, const char *target, bool on)
{
    int rc = -EINVAL;

    if (!strcmp(target, "rpu"))
    {
        rc = power_rpu(on);
    }
    else if (!strcmp(target, "sensor"))
    {
        rc = power_sensor(on);
    }
    else if (!strcmp(target, "led"))
    {
        rc = board_led_set(on);
    }

    if (rc == 0)
    {
        shell_print(shell, "gpio %s %s: OK", target, on ? "on" : "off");
    }
    else
    {
        shell_error(shell, "gpio %s %s: err=%d", target, on ? "on" : "off", rc);
    }
    return rc;
}

/* 개별 커맨드 */
static int cmd_gpio_rpu_on(const struct shell *sh, size_t a, char **v)
{
    ARG_UNUSED(a);
    ARG_UNUSED(v);
    return do_set(sh, "rpu", true);
}
static int cmd_gpio_rpu_off(const struct shell *sh, size_t a, char **v)
{
    ARG_UNUSED(a);
    ARG_UNUSED(v);
    return do_set(sh, "rpu", false);
}
static int cmd_gpio_sen_on(const struct shell *sh, size_t a, char **v)
{
    ARG_UNUSED(a);
    ARG_UNUSED(v);
    return do_set(sh, "sensor", true);
}
static int cmd_gpio_sen_off(const struct shell *sh, size_t a, char **v)
{
    ARG_UNUSED(a);
    ARG_UNUSED(v);
    return do_set(sh, "sensor", false);
}
static int cmd_gpio_led_on(const struct shell *sh, size_t a, char **v)
{
    ARG_UNUSED(a);
    ARG_UNUSED(v);
    return do_set(sh, "led", true);
}
static int cmd_gpio_led_off(const struct shell *sh, size_t a, char **v)
{
    ARG_UNUSED(a);
    ARG_UNUSED(v);
    return do_set(sh, "led", false);
}

/* 서브커맨드 트리 구성 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio_rpu,
                               SHELL_CMD(on, NULL, "diag gpio rpu on", cmd_gpio_rpu_on),
                               SHELL_CMD(off, NULL, "diag gpio rpu off", cmd_gpio_rpu_off),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio_sensor,
                               SHELL_CMD(on, NULL, "diag gpio sensor on", cmd_gpio_sen_on),
                               SHELL_CMD(off, NULL, "diag gpio sensor off", cmd_gpio_sen_off),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio_led,
                               SHELL_CMD(on, NULL, "diag gpio led on", cmd_gpio_led_on),
                               SHELL_CMD(off, NULL, "diag gpio led off", cmd_gpio_led_off),
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio_root,
                               SHELL_CMD(rpu, &sub_gpio_rpu, "RPU power enable", NULL),
                               SHELL_CMD(sensor, &sub_gpio_sensor, "Sensor power enable", NULL),
                               SHELL_CMD(led, &sub_gpio_led, "Status LED control", NULL),
                               SHELL_SUBCMD_SET_END);

/************** 가속소 센서 쉘 *********** */
static int cmd_imu_init(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    int rc = lsm6dso_init(LSM6DSO_FS_4G); /* 기본 ±4g */
    shell_print(shell, "lsm6dso_init: rc=%d", rc);
    return rc;
}

static int cmd_imu_once(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    lsm6dso_stats_t st = {0};
    int rc = lsm6dso_capture_once(&st);
    shell_print(shell, "rc=%d, n=%u, WHO=0x%02X, WTM=%d", rc, st.n, st.whoami, st.wtm_reached);

    if (st.n > 0)
    {
        shell_print(shell, "ALL  PEAK (x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                    st.peak_ms2_x100[0], st.peak_ms2_x100[1], st.peak_ms2_x100[2]);
        shell_print(shell, "ALL  RMS  (x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                    st.rms_ms2_x100[0], st.rms_ms2_x100[1], st.rms_ms2_x100[2]);
        shell_print(shell, "10-1000Hz PEAK(x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                    st.bl_peak_ms2_x100[0], st.bl_peak_ms2_x100[1], st.bl_peak_ms2_x100[2]);
        shell_print(shell, "10-1000Hz RMS (x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                    st.bl_rms_ms2_x100[0], st.bl_rms_ms2_x100[1], st.bl_rms_ms2_x100[2]);
    }
    return rc;
}

static int cmd_imu_loop(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);

    const uint32_t duration_ms = 10 * 1000;  /* 총 10초 */
    const uint32_t interval_ms = 500;        /* 0.5초 간격 */
    uint32_t t_start = k_uptime_get_32();
    uint32_t t_next  = t_start;

    while ((k_uptime_get_32() - t_start) < duration_ms) {
        uint32_t t_iter = k_uptime_get_32();

        lsm6dso_stats_t st = {0};
        int rc = lsm6dso_capture_once(&st);

        shell_print(shell, "rc=%d, n=%u, WHO=0x%02X, WTM=%d", rc, st.n, st.whoami, st.wtm_reached);
        if (st.n > 0) {
            shell_print(shell, "ALL  PEAK (x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                st.peak_ms2_x100[0], st.peak_ms2_x100[1], st.peak_ms2_x100[2]);
            shell_print(shell, "ALL  RMS  (x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                st.rms_ms2_x100[0],  st.rms_ms2_x100[1],  st.rms_ms2_x100[2]);
            shell_print(shell, "10-1000Hz PEAK(x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                st.bl_peak_ms2_x100[0], st.bl_peak_ms2_x100[1], st.bl_peak_ms2_x100[2]);
            shell_print(shell, "10-1000Hz RMS (x,y,z) = (%d,%d,%d) x0.01 m/s^2",
                st.bl_rms_ms2_x100[0],  st.bl_rms_ms2_x100[1],  st.bl_rms_ms2_x100[2]);
        }

        /* 0.5초 주기 정렬(측정 시간 포함하여 남은 시간만큼 sleep) */
        t_next += interval_ms;
        uint32_t now = k_uptime_get_32();
        if ((int32_t)(t_next - now) > 0) {
            k_sleep(K_MSEC(t_next - now));
        } else {
            /* 지연이 길어졌으면 다음 슬롯으로 보정 */
            t_next = now;
        }
    }
    return 0;
}

static int cmd_imu_regs(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    return lsm6dso_dump_regs(shell);
}

/* 쉘 서브커맨드 등록 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu,
                               SHELL_CMD(who, NULL, "LSM6DSO WHO_AM_I check", cmd_imu_who),
                               SHELL_CMD(init, NULL, "LSM6DSO init (ODR=3.33k, FS=±4g)", cmd_imu_init),
                               SHELL_CMD(once, NULL, "Capture burst -> peak/rms", cmd_imu_once),
                               SHELL_CMD(regs, NULL, "Dump key IMU/FIFO registers", cmd_imu_regs),
                               SHELL_CMD(loop, NULL, "10s, every 0.5s capture+print", cmd_imu_loop),
                               SHELL_SUBCMD_SET_END);

/* 서브커맨드 집합 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_diag,
                               SHELL_CMD(echo, NULL, "echo <text...> (UART RX check)", cmd_echo),
                               SHELL_CMD(log, NULL, "diag log [on|off] (show if no arg)", cmd_log_sw),
                               SHELL_CMD(ble, &sub_ble, "BLE controls", NULL),
                               SHELL_CMD(i2c, NULL, "I2C bus scan (0x03..0x77)", cmd_i2c_scan),
                               SHELL_CMD(dip, NULL, "Read DIP(TCA9534) and parse", cmd_dip_read),
                               SHELL_CMD(ntc, NULL, "Read NTC on AIN1 and print temperature", cmd_ntc),
                               SHELL_CMD(gpio, &sub_gpio_root, "GPIO controls", NULL),
                               SHELL_CMD(imu, &sub_imu, "IMU LSM6DSO test", NULL),
                               SHELL_SUBCMD_SET_END);

/* 루트 커맨드 등록 */
SHELL_CMD_REGISTER(diag, &sub_diag, "Diagnostics commands", NULL);

#endif /* IS_ENABLED(CONFIG_SHELL) */
