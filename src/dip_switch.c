/* dip_switch.c */
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "gpio_if.h"     /* power_rpu(true/false) */
#include "dip_switch.h"  /* struct dip_bits 선언부 */

/** @file dip_switch.c
 * @brief I2C I/O 확장기(TCA9534)를 사용하여 DIP 스위치 값을 읽고 해석하는 함수 구현.
 */

LOG_MODULE_REGISTER(dip, LOG_LEVEL_INF);

/* -------- TCA9534 (DIP I/O Expander) -------- */
/** @brief TCA9534 I2C 슬레이브 주소 */
#define TCA9534_ADDR   0x20
/** @brief 입력 포트 레지스터 주소 */
#define REG_INPUT      0x00
/** @brief 출력 포트 레지스터 주소 */
#define REG_OUTPUT     0x01
/** @brief 극성 반전 레지스터 주소 */
#define REG_POLARITY   0x02
/** @brief 구성(Config) 레지스터 주소 */
#define REG_CONFIG     0x03

/** @brief I2C0 디바이스 구조체 포인터 */
static const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* 내부 I2C 유틸 */
/** @brief TCA9534 레지스터에 1바이트를 기록합니다.
 * @param reg 기록할 레지스터 주소
 * @param v 기록할 값
 * @retval 0 성공
 * @retval 음수값 I2C 통신 실패 시 Zephyr 에러 코드
 */
static inline int tca_wr(uint8_t reg, uint8_t v)
{
    uint8_t buf[2] = { reg, v };
    return i2c_write(i2c0_dev, buf, 2, TCA9534_ADDR);
}

/** @brief TCA9534 레지스터에서 1바이트를 읽습니다.
 * @param reg 읽을 레지스터 주소
 * @param v 읽은 값이 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 I2C 통신 실패 시 Zephyr 에러 코드
 */
static inline int tca_rd(uint8_t reg, uint8_t *v)
{
    int r = i2c_write(i2c0_dev, &reg, 1, TCA9534_ADDR);
    if (r) return r;
    return i2c_read(i2c0_dev, v, 1, TCA9534_ADDR);
}

/* 비트 해석 */
/** @brief DIP 스위치 Raw 값을 dip_bits 구조체로 해석합니다.
 * @param v DIP 스위치의 Raw 값 (8비트)
 * @param o 해석된 비트들이 저장될 dip_bits 구조체 포인터
 */
static inline void parse_bits(uint8_t v, struct dip_bits *o)
{
    if (!o) return;
    o->model    = (v & 0x07);       /**< 비트 [2:0] */
    o->mode_sub = (v >> 3) & 0x01;  /**< 비트 [3] */
    o->phy      = (v >> 4) & 0x01;  /**< 비트 [4] */
    o->period   = (v >> 5) & 0x01;  /**< 비트 [5] */
    o->pwr      = (v >> 6) & 0x01;  /**< 비트 [6] */
    o->legacy   = (v >> 7) & 0x01;  /**< 비트 [7] */
}

/** @brief DIP 스위치 값을 읽고 해석합니다.
 *
 * 이 함수는 RPU 전원을 켠 후 TCA9534를 입력 모드로 설정하고,
 * 간단한 디바운싱을 거쳐 DIP 스위치의 Raw 값과 해석된 값을 반환합니다.
 *
 * @param raw DIP 스위치의 Raw 값 (8비트)이 저장될 포인터 (NULL 허용)
 * @param parsed 해석된 비트들이 저장될 dip_bits 구조체 포인터 (NULL 허용)
 * @retval 0 성공
 * @retval -ENODEV I2C 디바이스가 준비되지 않았을 때
 * @retval 음수값 I2C 통신 실패 시 Zephyr 에러 코드
 */
int Get_Switch(uint8_t *raw, struct dip_bits *parsed)
{
    /* 0) 전원 인가 (RPU rail) */
    power_rpu(true);
    k_msleep(5); /* 보드 기준 안정화 여유 */

    /* 1) I2C 준비 */
    if (!device_is_ready(i2c0_dev)) {
        LOG_ERR("i2c0 not ready");
        /* 전원은 다시 꺼줌 */
        power_rpu(false);
        return -ENODEV;
    }

    /* 2) TCA9534를 입력 모드로 설정 (REG_CONFIG = 0xFF), 극성 반전 안함 (REG_POLARITY = 0x00) */
    int r = tca_wr(REG_CONFIG, 0xFF);
    if (r) { LOG_ERR("TCA CFG write: %d", r); power_rpu(false); return r; }
    r = tca_wr(REG_POLARITY, 0x00);
    if (r) { LOG_ERR("TCA POL write: %d", r); power_rpu(false); return r; }

    /* 3) 입력 2회 읽어 간단 디바운싱 */
    uint8_t v1=0, v2=0;
    r = tca_rd(REG_INPUT, &v1);             if (r) { LOG_ERR("TCA IN rd(1): %d", r); power_rpu(false); return r; }
    k_busy_wait(200); /* 0.2ms 대기 */
    r = tca_rd(REG_INPUT, &v2);             if (r) { LOG_ERR("TCA IN rd(2): %d", r); power_rpu(false); return r; }

    /* 간단 디바운싱: 두 번 읽은 값이 같으면 그 값을, 다르면 비트 AND (스위치가 High로 튀는 경우 방지) */
    uint8_t v = (v1 == v2) ? v1 : (v1 & v2);

    /* 4) Raw 값 및 파싱된 값 저장 */
    if (raw)    *raw = v;
    if (parsed) parse_bits(v, parsed);

    /* 로그 출력 (디버그 용) */
    struct dip_bits log_bits;
    parse_bits(v, &log_bits);
    LOG_INF("DIP RAW=0x%02X | model=%u mode=%u phy=%u period=%u pwr=%u legacy=%u",
            v, log_bits.model, log_bits.mode_sub, log_bits.phy, log_bits.period, log_bits.pwr, log_bits.legacy);

    /* 5) 전원 해제 */
    power_rpu(false);
    return 0;
}

/** @brief DIP 스위치 값을 읽고 Raw 값만 반환합니다.
 * @param out_raw DIP 스위치의 Raw 값 (8비트)이 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 읽기 실패 시 Zephyr 에러 코드
 */
int dip_read_u8(uint8_t *out_raw)
{
    uint8_t raw = 0;
    int r = Get_Switch(&raw, NULL);
    if (r == 0 && out_raw) *out_raw = raw;
    return r;
}

/** @brief DIP 스위치 Raw 값을 dip_bits 구조체로 해석하여 반환합니다.
 * @param v DIP 스위치의 Raw 값 (8비트)
 * @return 해석된 dip_bits 구조체
 */
struct dip_bits parse_dip(uint8_t v)
{
    struct dip_bits b;
    parse_bits(v, &b);
    return b;
}
