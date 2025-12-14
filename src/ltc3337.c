#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "ltc3337.h"

LOG_MODULE_REGISTER(ltc3337, LOG_LEVEL_INF);

#define LTC3337_ADDR 0x64

/* 레지스터 서브어드레스 */
#define LTC3337_REG_A 0x01u
#define LTC3337_REG_B 0x02u
#define LTC3337_REG_C 0x03u
#define LTC3337_REG_D 0x04u
#define LTC3337_REG_E 0x05u
#define LTC3337_REG_F 0x06u
#define LTC3337_REG_G 0x07u
#define LTC3337_REG_H 0x08u

/* C 레지스터 비트 */
#define LTC3337_C_OVERFLOW_FAULT_BIT 0
#define LTC3337_C_COULOMB_ALARM_BIT 1
#define LTC3337_C_COLD_ALARM_BIT 2
#define LTC3337_C_HOT_ALARM_BIT 3
#define LTC3337_C_IPK_SHIFT 5 /* C[7:5] */

/* A 레지스터 필드 */
#define LTC3337_A_M_MASK 0x000Fu /* A[3:0] */

/* ADC 변환 (1카운트 = 1.465mV) */
#define LTC3337_V_LSB_UV 1465u

/* I2C 디바이스 */
static const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* 부팅 시 결정되는 설정 값 캐시 */
static uint8_t g_ipk = 0;
static uint8_t g_m = 0;
static uint32_t g_fs_mah = 0;

/* 16비트 레지스터 읽기(LSB-first) */
static int ltc3337_reg_read16(uint8_t subaddr, uint16_t *out)
{
    uint8_t rx[2];
    int ret;

    if (out == NULL) {
        return -EINVAL;
    }

    ret = i2c_write_read(i2c1, LTC3337_ADDR, &subaddr, 1, rx, sizeof(rx));
    if (ret) {
        return ret;
    }

    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return 0;
}

/* 16비트 레지스터 쓰기(LSB-first) */
static int ltc3337_reg_write16(uint8_t subaddr, uint16_t val)
{
    uint8_t tx[3];

    tx[0] = subaddr;
    tx[1] = (uint8_t)(val & 0xFFu);
    tx[2] = (uint8_t)((val >> 8) & 0xFFu);

    return i2c_write(i2c1, tx, sizeof(tx), LTC3337_ADDR);
}

/* 12비트 ADC 카운트를 uV로 변환 */
static uint32_t ltc3337_adc_count_to_uv(uint16_t reg_val)
{
    uint16_t count12 = reg_val & 0x0FFFu;
    return (uint32_t)count12 * (uint32_t)LTC3337_V_LSB_UV;
}

/* IPK 코드에 따라 M/FS(mAh) 결정(QBAT=17000mAh 기준) */
/* 3.6V 리튬 D셀을 19,000mAh로 본다면 각 fs를 동일한 비율(19000/17000 ≈ 1.118)로 조정 */
static void ltc3337_choose_m_fs(uint8_t ipk, uint8_t *m_out, uint32_t *fsmah_out)
{
    uint8_t m = 3;
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

/* IPK 코드를 IPEAK(mA)로 변환 */
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

    /* C 레지스터에서 IPK 스트랩 확인 */
    ret = ltc3337_reg_read16(LTC3337_REG_C, &reg_c);
    if (ret) {
        return ret;
    }

    g_ipk = (uint8_t)((reg_c >> LTC3337_C_IPK_SHIFT) & 0x7u);

    /* M/FS 계산 */
    ltc3337_choose_m_fs(g_ipk, &g_m, &g_fs_mah);

    /* A: A[3:0]=M, A[15:8]=0xFF(알람 임계 최대 유지) */
    reg_a = (uint16_t)(0xFF00u | (g_m & (uint8_t)LTC3337_A_M_MASK));
    ret = ltc3337_reg_write16(LTC3337_REG_A, reg_a);
    if (ret) {
        return ret;
    }

    /* 부팅 시 적산 카운터 초기화 - 값 유지시 실행하지 않아야 함 */
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

    /* 출력 버퍼 초기화(부분 실패 시 이전 값 잔류 방지) */
    *st = (struct ltc3337_status){0};

    /* 필수 레지스터 */
    ret = ltc3337_reg_read16(LTC3337_REG_A, &st->reg_a);
    if (ret) return ret;

    ret = ltc3337_reg_read16(LTC3337_REG_B, &st->reg_b);
    if (ret) return ret;

    ret = ltc3337_reg_read16(LTC3337_REG_C, &st->reg_c);
    if (ret) return ret;

    /* 선택 레지스터(전압 등). 실패해도 0 유지 후 진행 */
    (void)ltc3337_reg_read16(LTC3337_REG_D, &st->reg_d);
    (void)ltc3337_reg_read16(LTC3337_REG_E, &st->reg_e);
    (void)ltc3337_reg_read16(LTC3337_REG_F, &st->reg_f);
    (void)ltc3337_reg_read16(LTC3337_REG_G, &st->reg_g);
    (void)ltc3337_reg_read16(LTC3337_REG_H, &st->reg_h);

    /* IPK/M/FS 계산 */
    st->ipk_code = (uint8_t)((st->reg_c >> LTC3337_C_IPK_SHIFT) & 0x7u);
    st->qlsb_code = (uint8_t)(st->reg_a & LTC3337_A_M_MASK);
    {
        uint8_t calc_m = 0;
        uint32_t fs_mah = 0;
        ltc3337_choose_m_fs(st->ipk_code, &calc_m, &fs_mah);
        g_m = calc_m;
        g_fs_mah = fs_mah;
        st->fs_mah = fs_mah;
    }

    /* 알람 플래그 */
    st->flags.overflow_fault = (st->reg_c & BIT(LTC3337_C_OVERFLOW_FAULT_BIT)) != 0u;
    st->flags.coulomb_alarm  = (st->reg_c & BIT(LTC3337_C_COULOMB_ALARM_BIT))  != 0u;
    st->flags.cold_alarm     = (st->reg_c & BIT(LTC3337_C_COLD_ALARM_BIT))     != 0u;
    st->flags.hot_alarm      = (st->reg_c & BIT(LTC3337_C_HOT_ALARM_BIT))      != 0u;

    /* I2C로만 확인하는 bat_ok 프록시 */
    st->bat_ok = !st->flags.overflow_fault;

    /* 적산 카운터를 mAh로 변환 (0..65535 -> 0..FS) */
    st->used_mah = (uint32_t)(((uint64_t)st->reg_b * (uint64_t)st->fs_mah + 32767u) / 65535u);
    st->mah_used_fs = st->used_mah;

    /* % * 100 (예: 12.34% -> 1234) */
    st->used_pct_x100 = (st->fs_mah > 0u) ?
        (uint32_t)(((uint64_t)st->used_mah * 10000u + (st->fs_mah / 2u)) / st->fs_mah) : 0u;

    /* 전압(uV) */
    st->vbat_in_on_uv   = ltc3337_adc_count_to_uv(st->reg_d);
    st->vbat_in_off_uv  = ltc3337_adc_count_to_uv(st->reg_e);
    st->vbat_out_on_uv  = ltc3337_adc_count_to_uv(st->reg_f);
    st->vbat_out_off_uv = ltc3337_adc_count_to_uv(st->reg_g);

    /* 다이 온도 코드 */
    st->die_temp_code = (uint8_t)((st->reg_c >> 8) & 0xFFu);

    /* 임피던스 근사: Z ≈ dV(uV) / I(mA) */
    {
        uint32_t ipeak_ma = ltc3337_ipk_to_ipeak_ma(st->ipk_code);
        int32_t dv_uv = (int32_t)st->vbat_in_off_uv - (int32_t)st->vbat_in_on_uv;
        st->z_uohm = (ipeak_ma > 0u) ? (dv_uv / (int32_t)ipeak_ma) : 0;
    }

    return 0;
}
