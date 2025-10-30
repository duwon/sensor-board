/* dip_switch.h */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file dip_switch.h
 * @brief DIP 스위치 값을 읽고 해석하기 위한 인터페이스 정의 파일
 */

/** @struct dip_bits
 * @brief 8비트 DIP 스위치 값을 용도별 필드로 해석한 구조체
 */
struct dip_bits {
    uint8_t model;     /**< [2:0] 모델/디바이스 ID 비트 (3비트) */
    uint8_t mode_sub;  /**< [3] 모드/서브 모드 비트 (1비트) */
    uint8_t phy;       /**< [4] PHY/하드웨어 설정 비트 (1비트) */
    uint8_t period;    /**< [5] 주기/전송 간격 비트 (1비트) */
    uint8_t pwr;       /**< [6] 전원 관련 설정 비트 (1비트) */
    uint8_t legacy;    /**< [7] 레거시 설정 비트 (1비트) */
};

/** @brief DIP 스위치 값을 I2C로 읽고 해석합니다.
 * @see Get_Switch
 * @param raw DIP 스위치의 Raw 값 (8비트)이 저장될 포인터 (NULL 허용)
 * @param parsed 해석된 dip_bits 구조체가 저장될 포인터 (NULL 허용)
 * @retval 0 성공
 * @retval 음수값 I2C 통신 실패 시 Zephyr 에러 코드
 */
int Get_Switch(uint8_t *raw, struct dip_bits *parsed);

/** @brief DIP 스위치 값을 I2C로 읽고 Raw 값만 반환합니다.
 * @see dip_read_u8
 * @param out_raw DIP 스위치의 Raw 값 (8비트)이 저장될 포인터
 * @retval 0 성공
 * @retval 음수값 읽기 실패 시 Zephyr 에러 코드
 */
int dip_read_u8(uint8_t *out_raw);

/** @brief DIP 스위치 Raw 값을 dip_bits 구조체로 해석하여 반환합니다.
 * @see parse_dip
 * @param v DIP 스위치의 Raw 값 (8비트)
 * @return 해석된 dip_bits 구조체
 */
struct dip_bits parse_dip(uint8_t v);

#ifdef __cplusplus
}
#endif
/* end of dip_switch.h */