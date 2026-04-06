#pragma once
/**
 * @file ble_adv.h
 * @brief BLE 광고 데이터 구조와 제어 인터페이스 선언
 */
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
    uint8_t structure_version;
    uint8_t model_code;
	uint8_t device_status;
    uint8_t error_info;
    int8_t mcu_temperature;
    uint8_t battery_percent;
    uint8_t value_presence_mask;
    int32_t sensor_value_1;
    int32_t sensor_value_2;
    int32_t sensor_value_3;
    int32_t sensor_value_4;
    int32_t sensor_value_5;
    int32_t sensor_value_6;
};

struct Status
{
	uint8_t  Dipsw;						// 부팅 후 읽은 딥스위치 값
	uint8_t  Led_Cnt;					// led blink 횟수
	uint16_t Led_Interval;				// LED 브링크 간격
	uint8_t  Sleep_Sec;					// sleep 시간
	uint16_t Complete;					// ble 송신완료 후 sleep 진입확인 flag
	uint16_t Model;	
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
int ble_adv_ext_init(void);
void ble_setup (uint8_t phy, uint8_t scan);

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
