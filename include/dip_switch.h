/* dip_switch.h */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dip_bits {
    uint8_t model;     /* [2:0] */
    uint8_t mode_sub;  /* [3]   */
    uint8_t phy;       /* [4]   */
    uint8_t period;    /* [5]   */
    uint8_t pwr;       /* [6]   */
    uint8_t legacy;    /* [7]   */
};

int Get_Switch(uint8_t *raw, struct dip_bits *parsed);
int dip_read_u8(uint8_t *out_raw);
struct dip_bits parse_dip(uint8_t v);

#ifdef __cplusplus
}
#endif
/* end of dip_switch.h */