
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief LTC3337 알람/상태 비트
     */
    struct ltc3337_flags
    {
        bool overflow_fault; /**< C[0] ripple-counter overflow/fault (latched) */
        bool coulomb_alarm;  /**< C[1] coulomb counter alarm */
        bool cold_alarm;     /**< C[2] cold temperature alarm */
        bool hot_alarm;      /**< C[3] hot temperature alarm */
    };

    /**
     * @brief LTC3337 배터리/전력 상태
     *
     * - used_mah / used_pct_x100: "누적 사용량"을 현장 스케일 FS(mAh) 기준으로 변환
     * - vbat_*_uv: 12-bit ADC 결과를 uV로 변환한 값
     * - z_uohm: Z ≈ (VBAT_IN(OFF)-VBAT_IN(ON))/IPEAK 를 micro-ohm 근사
     * - die_temp_code: 데이터시트의 온도 코드(변환식 필요 시 추가 가능)
     */
    struct ltc3337_status
    {
        bool bat_ok;          /**< I2C 기반 proxy OK  */
        uint8_t qlsb_code;    /**< prescaler M (A[3:0]) */
        uint32_t mah_used_fs; /**< 누적 사용량(mAh) - FS 기준 환산 */

        /* 설정/스케일 정보 */
        uint8_t ipk_code;       /**< IPK strap code (C[7:5]) */
        uint32_t fs_mah;        /**< Full-scale mAh (프로젝트 테이블 기준) */
        uint32_t used_mah;      /**< mah_used_fs와 동일 */
        uint32_t used_pct_x100; /**< 사용률 [%] * 100 (예: 1234 => 12.34%) */

        /* 알람/상태 */
        struct ltc3337_flags flags;

        /* 전압/온도/임피던스 */
        uint32_t vbat_in_on_uv; /* VBAT_IN을 부하 걸고 측정 한 값 */
        uint32_t vbat_in_off_uv; /* VBAT_IN을 무부하로 측정 한 값 */
        uint32_t vbat_out_on_uv; /* VBAT_OUT을 부하 걸고 측정 한 값 */
        uint32_t vbat_out_off_uv; /* VBAT_OUT을 무부하로 측정 한 값 */

        int32_t z_uohm;        /**< 배터리 임피던스 근사값 */
        uint8_t die_temp_code; /**< die temperature raw code (C[15:8]) */

        /* raw registers (디버깅/로깅용) */
        uint16_t reg_a;
        uint16_t reg_b;
        uint16_t reg_c;
        uint16_t reg_d;
        uint16_t reg_e;
        uint16_t reg_f;
        uint16_t reg_g;
        uint16_t reg_h;
    };

    /**
     * @brief LTC3337 초기화
     * @return 0 on success, negative errno on failure
     */
    int ltc3337_init(void);

    /**
     * @brief LTC3337 상태 읽기
     * @param[out] st 상태 구조체 포인터
     * @return 0 on success, negative errno on failure
     */
    int ltc3337_read_status(struct ltc3337_status *st);

#ifdef __cplusplus
}
#endif
