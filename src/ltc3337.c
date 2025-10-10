
#include "ltc3337.h"

static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

int ltc3337_init(void){
    if (!device_is_ready(i2c0)) return -ENODEV;
    return 0;
}

int ltc3337_read_status(struct ltc3337_status *st){
    if (!device_is_ready(i2c0)) return -ENODEV;
    uint8_t reg = 0x00, buf[2] = {0};
    int r = i2c_write_read(i2c0, LTC3337_ADDR, &reg, 1, buf, 2);
    if (r) return r;
    st->qlsb_code = buf[0];
    st->mah_used_fs = 0;
    return 0;
}
