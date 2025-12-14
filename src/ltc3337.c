#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "ltc3337.h"

LOG_MODULE_REGISTER(ltc3337, LOG_LEVEL_INF);

#define LTC3337_ADDR 0x64

/* Register sub-address map */
#define LTC3337_REG_A 0x01u
#define LTC3337_REG_B 0x02u
#define LTC3337_REG_C 0x03u
#define LTC3337_REG_D 0x04u
#define LTC3337_REG_E 0x05u
#define LTC3337_REG_F 0x06u
#define LTC3337_REG_G 0x07u
#define LTC3337_REG_H 0x08u

/* Register C bits */
#define LTC3337_C_OVERFLOW_FAULT_BIT   0
#define LTC3337_C_COULOMB_ALARM_BIT    1
#define LTC3337_C_COLD_ALARM_BIT       2
#define LTC3337_C_HOT_ALARM_BIT        3
#define LTC3337_C_IPK_SHIFT            5 /* C[7:5] */

/* Register A fields */
#define LTC3337_A_M_MASK               0x000Fu /* A[3:0] */

/* ADC */
#define LTC3337_V_LSB_UV 1465u /* 1.465mV per count => 1465uV */

/* I2C device */
static const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* Cached configuration derived at init */
static uint8_t  g_ipk = 0;
static uint8_t  g_m   = 0;
static uint32_t g_fs_mah = 0;

/**
 * @brief Read a 16-bit register from LTC3337 (LSB-first).
 */
static int ltc3337_reg_read16(uint8_t subaddr, uint16_t *out)
{
    uint8_t rx[2];
    int ret;

    if (out == NULL) {
        return -EINVAL;
    }

    ret = i2c_write(i2c1, &subaddr, 1, LTC3337_ADDR);
    if (ret) {
        return ret;
    }

    ret = i2c_read(i2c1, rx, sizeof(rx), LTC3337_ADDR);
    if (ret) {
        return ret;
    }

    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return 0;
}

/**
 * @brief Write a 16-bit register to LTC3337 (LSB-first).
 */
static int ltc3337_reg_write16(uint8_t subaddr, uint16_t val)
{
    uint8_t tx[3];

    tx[0] = subaddr;
    tx[1] = (uint8_t)(val & 0xFFu);
    tx[2] = (uint8_t)((val >> 8) & 0xFFu);

    return i2c_write(i2c1, tx, sizeof(tx), LTC3337_ADDR);
}

/**
 * @brief Convert 12-bit ADC count (bits[11:0]) to uV.
 */
static uint32_t ltc3337_adc_count_to_uv(uint16_t reg_val)
{
    uint16_t count12 = reg_val & 0x0FFFu;
    return (uint32_t)count12 * (uint32_t)LTC3337_V_LSB_UV;
}

/**
 * @brief Choose prescaler M and FS(mAh) from project spec table (QBAT=17,000mAh).
 */
static void ltc3337_choose_m_fs(uint8_t ipk, uint8_t *m_out, uint32_t *fsmah_out)
{
    /* defaults */
    uint8_t  m  = 3;
    uint32_t fs = 30542;

    switch (ipk & 0x7u) {
    case 0x0: m = 1; fs = 24434; break; /* 5mA  */
    case 0x1: m = 2; fs = 24434; break; /* 10mA */
    case 0x2: m = 3; fs = 18325; break; /* 15mA */
    case 0x3: m = 3; fs = 24434; break; /* 20mA */
    case 0x4: m = 3; fs = 30542; break; /* 25mA */
    case 0x5: m = 4; fs = 30542; break; /* 50mA */
    case 0x6: m = 5; fs = 22906; break; /* 75mA */
    case 0x7: m = 5; fs = 30542; break; /* 100mA */
    default:  m = 3; fs = 30542; break;
    }

    *m_out = m;
    *fsmah_out = fs;
}

/**
 * @brief Convert IPK code to IPEAK (mA).
 */
static uint32_t ltc3337_ipk_to_ipeak_ma(uint8_t ipk)
{
    switch (ipk & 0x7u) {
    case 0x0: return 5u;
    case 0x1: return 10u;
    case 0x2: return 15u;
    case 0x3: return 20u;
    case 0x4: return 25u;
    case 0x5: return 50u;
    case 0x6: return 75u;
    case 0x7: return 100u;
    default:  return 25u;
    }
}

int ltc3337_init(void)
{
    int ret;
    uint16_t reg_c;
    uint16_t reg_a;

    if (!device_is_ready(i2c1)) {
        return -ENODEV;
    }

    /* Read C to get IPK[2:0] */
    ret = ltc3337_reg_read16(LTC3337_REG_C, &reg_c);
    if (ret) {
        return ret;
    }

    g_ipk = (uint8_t)((reg_c >> LTC3337_C_IPK_SHIFT) & 0x7u);

    /* Choose M and FS */
    ltc3337_choose_m_fs(g_ipk, &g_m, &g_fs_mah);

    /*
     * Program A:
     *  - A[3:0] = M
     *  - A[15:8] alarm threshold keep 0xFF (max)
     */
    reg_a = (uint16_t)(0xFF00u | (g_m & (uint8_t)LTC3337_A_M_MASK));
    ret = ltc3337_reg_write16(LTC3337_REG_A, reg_a);
    if (ret) {
        return ret;
    }

    /* Optional: clear accumulated charge counter at boot */
    ret = ltc3337_reg_write16(LTC3337_REG_B, 0x0000u);
    if (ret) {
        return ret;
    }

    LOG_INF("LTC3337 init: IPK=%u, M=%u, FS=%u mAh", g_ipk, g_m, g_fs_mah);
    return 0;
}

int ltc3337_read_status(struct ltc3337_status *st)
{
    int ret;

    if (st == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(i2c1)) {
        return -ENODEV;
    }

    /* Clear output first (avoid stale fields on partial read failure) */
    *st = (struct ltc3337_status){0};

    /* Read essential registers */
    ret = ltc3337_reg_read16(LTC3337_REG_A, &st->reg_a);
    if (ret) return ret;

    ret = ltc3337_reg_read16(LTC3337_REG_B, &st->reg_b);
    if (ret) return ret;

    ret = ltc3337_reg_read16(LTC3337_REG_C, &st->reg_c);
    if (ret) return ret;

    /* Optional registers (voltage etc). If fail, keep 0 but don't abort. */
    (void)ltc3337_reg_read16(LTC3337_REG_D, &st->reg_d);
    (void)ltc3337_reg_read16(LTC3337_REG_E, &st->reg_e);
    (void)ltc3337_reg_read16(LTC3337_REG_F, &st->reg_f);
    (void)ltc3337_reg_read16(LTC3337_REG_G, &st->reg_g);
    (void)ltc3337_reg_read16(LTC3337_REG_H, &st->reg_h);

    /* Decode IPK and choose M/FS (keep aligned with board strap) */
    st->ipk_code = (uint8_t)((st->reg_c >> LTC3337_C_IPK_SHIFT) & 0x7u);
    ltc3337_choose_m_fs(st->ipk_code, &g_m, &g_fs_mah);

    /* M can be read from A too; prefer cached logic but expose both ways */
    st->qlsb_code = (uint8_t)(st->reg_a & LTC3337_A_M_MASK);
    st->fs_mah = g_fs_mah;

    /* Alarms / flags */
    st->flags.overflow_fault = (st->reg_c & BIT(LTC3337_C_OVERFLOW_FAULT_BIT)) != 0u;
    st->flags.coulomb_alarm  = (st->reg_c & BIT(LTC3337_C_COULOMB_ALARM_BIT))  != 0u;
    st->flags.cold_alarm     = (st->reg_c & BIT(LTC3337_C_COLD_ALARM_BIT))     != 0u;
    st->flags.hot_alarm      = (st->reg_c & BIT(LTC3337_C_HOT_ALARM_BIT))      != 0u;

    /* I2C-only proxy bat_ok (GPIO BATOUT_OK 있으면 상위에서 교체 권장) */
    st->bat_ok = !st->flags.overflow_fault;

    /* Convert charge counter -> mAh (0..65535 => 0..FS) */
    st->used_mah = (uint32_t)(((uint64_t)st->reg_b * (uint64_t)st->fs_mah + 32767u) / 65535u);
    st->mah_used_fs = st->used_mah;

    /* Percent * 100 (e.g., 12.34%) */
    st->used_pct_x100 = (st->fs_mah > 0u) ?
        (uint32_t)(((uint64_t)st->used_mah * 10000u + (st->fs_mah / 2u)) / st->fs_mah) : 0u;

    /* Voltages (uV) */
    st->vbat_in_on_uv   = ltc3337_adc_count_to_uv(st->reg_d);
    st->vbat_in_off_uv  = ltc3337_adc_count_to_uv(st->reg_e);
    st->vbat_out_on_uv  = ltc3337_adc_count_to_uv(st->reg_f);
    st->vbat_out_off_uv = ltc3337_adc_count_to_uv(st->reg_g);

    /* Die temperature raw code */
    st->die_temp_code = (uint8_t)((st->reg_c >> 8) & 0xFFu);

    /* Impedance approx: Z(uOhm) ≈ dV(uV)/I(mA) */
    {
        uint32_t ipeak_ma = ltc3337_ipk_to_ipeak_ma(st->ipk_code);
        int32_t dv_uv = (int32_t)st->vbat_in_off_uv - (int32_t)st->vbat_in_on_uv;
        st->z_uohm = (ipeak_ma > 0u) ? (dv_uv / (int32_t)ipeak_ma) : 0;
    }

    return 0;
}
