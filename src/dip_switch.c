
#include "dip_switch.h"

static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

int dip_read_u8(uint8_t *raw)
{
    if (!device_is_ready(i2c0)) return -ENODEV;
    uint8_t reg = 0x00;
    int r = i2c_write_read(i2c0, TCA9534_I2C_ADDR, &reg, 1, raw, 1);
    if (r) return r;
    return 0;
}

struct dip_bits parse_dip(uint8_t v)
{
    struct dip_bits b;
    b.model   = (v & 0x07);
    b.mode_sub= (v >> 3) & 0x01;
    b.phy     = (v >> 4) & 0x01;
    b.period  = (v >> 5) & 0x01;
    b.pwr     = (v >> 6) & 0x01;
    b.legacy  = (v >> 7) & 0x01;
    return b;
}
