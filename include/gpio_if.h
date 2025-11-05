#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

/** @file gpio_if.h
 * @brief 보드 GPIO 인터페이스 정의 파일
 */

/** @enum btn_evt_t
 * @brief 버튼 눌림 이벤트 유형
 */
typedef enum
{
    BTN_EVT_NONE = 0,  /**< 이벤트 없음 */
    BTN_EVT_SHORT = 1, /**< 짧게 눌림 */
    BTN_EVT_LONG = 2,  /**< 길게 눌림 */
} btn_evt_t;

/** @struct board_gpio
 * @brief 보드의 모든 GPIO 핀 구조체
 */
struct board_gpio
{
    const struct gpio_dt_spec led;       /**< LED 제어 핀 */
    const struct gpio_dt_spec btn;       /**< 사용자 버튼 핀 (Active-Low) */
    const struct gpio_dt_spec io_int;    /**< IO 인터럽트 출력 핀 */
    const struct gpio_dt_spec en_sensor; /**< 센서 전원 인에이블 핀 */
    const struct gpio_dt_spec en_rpu;    /**< RPU 전원 인에이블 핀 */
    const struct gpio_dt_spec soh_alarm; /**< SOH 알람 입력 핀 */
    const struct gpio_dt_spec soh_ok;    /**< SOH OK 입력 핀 */
};

/** @brief 보드의 모든 GPIO 핀들을 초기화하고 버튼 인터럽트를 설정합니다.
 * @see board_gpio_init
 * @retval 0 성공
 * @retval -ENODEV GPIO 포트 디바이스를 찾을 수 없을 때
 * @retval 음수값 GPIO 설정 또는 인터럽트 설정 실패 시 Zephyr 에러 코드
 */
int board_gpio_init(void);

/** @brief LED 핀의 상태를 설정합니다.
 * @see board_led_set
 * @param on true면 켜짐, false면 꺼짐
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int board_led_set(bool on);

/** @brief 버튼 핀의 현재 상태를 가져옵니다.
 * @see board_btn_get
 * @return 버튼이 눌렸으면 (Active-Low: 레벨 0) true, 아니면 false
 */
bool board_btn_get(void);

/** @brief io_int 핀에 지정된 시간 동안 펄스(Low->High)를 발생시킵니다.
 * @see io_int_pulse_us
 * @param usec 펄스를 Low 상태로 유지하는 시간 (마이크로초)
 */
void io_int_pulse_us(uint32_t usec);

/** @brief 센서 전원(en_sensor)을 제어합니다.
 * @see power_sensor
 * @param on true면 전원 켜짐(Active), false면 꺼짐(Inactive)
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int power_sensor(bool on);

/** @brief RPU 전원(en_rpu)을 제어합니다.
 * @see power_rpu
 * @param on true면 전원 켜짐(Active), false면 꺼짐(Inactive)
 * @retval 0 성공
 * @retval 음수값 실패 시 Zephyr 에러 코드
 */
int power_rpu(bool on);

/** @brief SOH_ALARM 핀의 상태를 가져옵니다.
 * @see soh_alarm_get
 * @return 핀 레벨 상태 (0 또는 1)
 */
bool soh_alarm_get(void);

/** @brief SOH_OK 핀의 상태를 가져옵니다.
 * @see soh_ok_get
 * @return 핀 레벨 상태 (0 또는 1)
 */
bool soh_ok_get(void);

/** @brief 발생한 버튼 이벤트 상태를 가져오고 상태를 초기화합니다.
 * @see Get_BtnStatus
 * @return 마지막으로 발생한 버튼 이벤트 (btn_evt_t). 이벤트가 없으면 BTN_EVT_NONE.
 */
btn_evt_t Get_BtnStatus(void);
