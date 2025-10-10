#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "gpio_if.h"     /* power_rpu(true/false) */
#include "dip_switch.h"  /* struct dip_bits 선언부 (아래 주석 참고) */

LOG_MODULE_REGISTER(dip, LOG_LEVEL_INF);

/* -------- TCA9534 (DIP I/O Expander) -------- */
#define TCA9534_ADDR   0x20
#define REG_INPUT      0x00
#define REG_OUTPUT     0x01
#define REG_POLARITY   0x02
#define REG_CONFIG     0x03

static const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* 내부 I2C 유틸 */
static inline int tca_wr(uint8_t reg, uint8_t v)
{
    uint8_t buf[2] = { reg, v };
    return i2c_write(i2c0_dev, buf, 2, TCA9534_ADDR);
}
static inline int tca_rd(uint8_t reg, uint8_t *v)
{
    int r = i2c_write(i2c0_dev, &reg, 1, TCA9534_ADDR);
    if (r) return r;
    return i2c_read(i2c0_dev, v, 1, TCA9534_ADDR);
}

/* 비트 해석 */
static inline void parse_bits(uint8_t v, struct dip_bits *o)
{
    if (!o) return;
    o->model    = (v & 0x07);
    o->mode_sub = (v >> 3) & 0x01;
    o->phy      = (v >> 4) & 0x01;
    o->period   = (v >> 5) & 0x01;
    o->pwr      = (v >> 6) & 0x01;
    o->legacy   = (v >> 7) & 0x01;
}

/* -------------------------------------------------
 * Get_Switch()
 *  - EN_RPU 전원 ON → TCA9534 입력모드 설정 → DIP 읽기
 *  - raw/parsed 모두 NULL 가능
 * 반환: 0=성공, 그 외 음수 errno
 * -------------------------------------------------*/
int Get_Switch(uint8_t *raw, struct dip_bits *parsed)
{
    /* 0) 전원 인가 (RPU rail) */
    power_rpu(true);
    k_msleep(5); /* 보드 기준 안정화 여유 (필요 시 10~20ms로 증가) */

    /* 1) I2C 준비 */
    if (!device_is_ready(i2c0_dev)) {
        LOG_ERR("i2c0 not ready");
        return -ENODEV;
    }

    /* 2) 모든 핀 입력, 극성 그대로 */
    int r = tca_wr(REG_CONFIG, 0xFF);
    if (r) { LOG_ERR("TCA CFG write: %d", r); return r; }
    r = tca_wr(REG_POLARITY, 0x00);
    if (r) { LOG_ERR("TCA POL write: %d", r); return r; }

    /* 3) 입력 2회 읽어 간단 디바운싱 */
    uint8_t v1=0, v2=0;
    r = tca_rd(REG_INPUT, &v1);             if (r) { LOG_ERR("TCA IN rd(1): %d", r); return r; }
    k_busy_wait(200); /* 0.2ms */
    r = tca_rd(REG_INPUT, &v2);             if (r) { LOG_ERR("TCA IN rd(2): %d", r); return r; }
    uint8_t v = (v1 == v2) ? v1 : (v1 & v2);

    if (raw)    *raw = v;
    if (parsed) parse_bits(v, parsed);

    LOG_INF("DIP RAW=0x%02X  model=%u mode=%u phy=%u period=%u pwr=%u legacy=%u",
            v, v & 7, (v>>3)&1, (v>>4)&1, (v>>5)&1, (v>>6)&1, (v>>7)&1);

    return 0;
}

/* -------------------------------------------------
 * DIP스위치 단순 raw 읽기
 * -------------------------------------------------*/
int dip_read_u8(uint8_t *out_raw)
{
    uint8_t raw = 0;
    int r = Get_Switch(&raw, NULL);
    if (r == 0 && out_raw) *out_raw = raw;
    return r;
}

/* DIP 스위치 parser */
struct dip_bits parse_dip(uint8_t v)
{
    struct dip_bits b;
    parse_bits(v, &b);
    return b;
}
