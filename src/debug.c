#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "debug.h"
#include "dip_switch.h"   /* Get_Switch() */
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
    if (!device_is_ready(i2c0_dev)) {
        LOG_ERR("i2c0 not ready");
        return -ENODEV;
    }
    LOG_INF("I2C0 scan start");

    uint8_t reg = 0x00;
    uint8_t val = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        /* 레지스터 없는 디바이스도 있어 읽기만 시도 → 많은 경우 NACK 허용 */
        int r = i2c_write_read(i2c0_dev, addr, &reg, 1, &val, 1);
        if (r == 0) {
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
#define LSM6DSO_ADDR  0x6A
#define WHO_AM_I      0x0F
#define LSM6DSO_ID    0x6C

int lsm6dso_probe(void)
{
    if (!device_is_ready(i2c0_dev)) return -ENODEV;

    uint8_t reg = WHO_AM_I, id = 0;
    int r = i2c_write_read(i2c0_dev, LSM6DSO_ADDR, &reg, 1, &id, 1);
    if (r) {
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
    for (size_t i=1; i<argc; i++) {
        shell_fprintf(sh, SHELL_NORMAL, "%s%s", argv[i], (i+1<argc)?" ":"");
    }
    shell_fprintf(sh, SHELL_NORMAL, "\n");
    return 0;
}

static int cmd_i2c_scan(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    int r = i2c_bus_scan();
    shell_print(sh, "i2c_scan ret=%d", r);
    return r;
}

static int cmd_imu_who(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    int r = lsm6dso_probe();
    shell_print(sh, "lsm6dso ret=%d", r);
    return r;
}

static int cmd_dip_read(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    uint8_t raw=0;
    struct dip_bits d={0};
    extern int Get_Switch(uint8_t *raw, struct dip_bits *parsed);
    int r = Get_Switch(&raw, &d);
    if (r==0) {
        shell_print(sh, "DIP RAW=0x%02X model=%u mode=%u phy=%u period=%u pwr=%u legacy=%u",
                    raw, d.model, d.mode_sub, d.phy, d.period, d.pwr, d.legacy);
    } else {
        shell_error(sh, "Get_Switch fail: %d", r);
    }
    return r;
}

static int cmd_log_sw(const struct shell *sh, size_t argc, char **argv)
{
    if (argc == 1) {
        shell_print(sh, "diag log is %s", app_diag_log_is_enabled() ? "ON" : "OFF");
        return 0;
    }
    if (strcmp(argv[1], "on") == 0) {
        app_diag_log_enable(true);
        shell_print(sh, "diag log ON");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0) {
        app_diag_log_enable(false);
        shell_print(sh, "diag log OFF");
        return 0;
    }
    shell_error(sh, "usage: diag log [on|off]");
    return -EINVAL;
}

/* 서브커맨드 집합 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_diag,
    SHELL_CMD(echo,     NULL, "echo <text...> (UART RX check)",   cmd_echo),
    SHELL_CMD(i2c-scan, NULL, "I2C bus scan (0x03..0x77)",        cmd_i2c_scan),
    SHELL_CMD(imu-who,  NULL, "LSM6DSO WHO_AM_I check",           cmd_imu_who),
    SHELL_CMD(dip-read, NULL, "Read DIP(TCA9534) and parse",      cmd_dip_read),
    SHELL_CMD(log,      NULL, "diag log [on|off]  (show if no arg)", cmd_log_sw),
    SHELL_SUBCMD_SET_END
);

/* 루트 커맨드 등록 */
SHELL_CMD_REGISTER(diag, &sub_diag, "Diagnostics commands", NULL);

#endif /* IS_ENABLED(CONFIG_SHELL) */