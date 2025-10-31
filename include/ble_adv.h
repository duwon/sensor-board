#pragma once
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/sys/byteorder.h>
#include <stdint.h>

/**
 * @brief BLE Advertising Data - Manufacturer Specific Data Payload 구조체
 *
 * 이 구조체는 AD Type(0xFF) 뒤에 오는 Manufacturer Specific Data의 내용입니다.
 * 총 길이: 33 Bytes (Index 5 ~ 37)
 */
struct __packed sensor_adv_data_t {
    /* ---------------- AD Structure 2: 헤더 및 상태 관리 (9 Bytes) ---------------- */
    /** @brief [Index 5-6] Company ID (Little Endian)
     * @details Bluetooth SIG Company ID (0xFFFF) 
     */
    uint16_t company_id;

    /** @brief [Index 7] 데이터 구조 버전
     * @details 0x01 
     */
    uint8_t structure_version;

    /** @brief [Index 8] 센서 모델 코드
     * @details 센서 종류를 나타내는 숫자 코드 (예: 0x01=T100) 
     */
    uint8_t model_code;

    /** @brief [Index 9] 장치 상태 (Device Status)
     * @details 0x00: 정상, 0x01: 오류 (Bitfield: 센서 연결, SOH 연결 등)
     */
    uint8_t device_status;

    /** @brief [Index 10] 오류 상세 정보 (Error Info)
     * @details Runtime 에러 발생 시 추가 정보 
     */
    uint8_t error_info;

    /** @brief [Index 11] MCU 온도
     * @details Signed 8-bit Integer (소수점 이하 버림) 
     */
    int8_t mcu_temperature;

    /** @brief [Index 12] 배터리 잔량
     * @details 0 ~ 100 (%) 
     */
    uint8_t battery_percent;

    /** @brief [Index 13] 값 유효성 마스크 (Value Presence Mask)
     * @details Sensor Value 1~6 중 어떤 값이 유효한지 나타내는 비트마스크 
     */
    uint8_t value_presence_mask;

    /* ---------------- AD Structure 2: 데이터 페이로드 (24 Bytes) ---------------- */
    /** @brief [Index 14-17] 센서 값 1
     * @details 32-bit Signed Integer, 값에 따라 Value=실제 값 x100 
     */
    int32_t sensor_value_1;

    /** @brief [Index 18-21] 센서 값 2
     * @details 32-bit Signed Integer 
     */
    int32_t sensor_value_2;

    /** @brief [Index 22-25] 센서 값 3
     * @details 32-bit Signed Integer 
     */
    int32_t sensor_value_3;

    /** @brief [Index 26-29] 센서 값 4
     * @details 32-bit Signed Integer 
     */
    int32_t sensor_value_4;

    /** @brief [Index 30-33] 센서 값 5
     * @details 32-bit Signed Integer 
     */
    int32_t sensor_value_5;

    /** @brief [Index 34-37] 센서 값 6
     * @details 32-bit Signed Integer
     */
    int32_t sensor_value_6;
};

/** @enum ble_init_t
 * @brief ble 초기화 유형
 */
typedef enum
{
    BLE_INIT = 0,  /**<  BLE 초기화 1번 실행 */
    BLE_SCAN_RESPONSE = 1, /**< 스캔 응답 */
    BTN_ADV = 2,  /**< 확장 광고 */
} ble_init_t;


/** @brief BLE 광고 모듈 초기화
 *
 * 블루투스 스택을 활성화하고 확장 광고 세트를 생성합니다.
 *
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Init_Ble(ble_init_t init_type);

/** @brief Manufacturer 데이터를 설정하고 1회성 광고를 수행합니다.
 *
 * 광고 데이터를 설정하고, Ble_Start() -> k_sleep() -> Ble_Stop()
 * 과정을 내부적으로 처리합니다.
 *
 * @param mfg Manufacturer Specific Data 페이로드 시작 주소.
 * @param mfg_len 페이로드 길이.
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Tx_Ble(const uint8_t *mfg, size_t mfg_len);

/** @brief 확장 광고를 중지합니다.
 *
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Ble_Stop(void);

/** @brief 확장 광고를 시작합니다.
 *
 * @return 0이면 성공, 음수이면 오류 코드.
 */
int Ble_Start(void);