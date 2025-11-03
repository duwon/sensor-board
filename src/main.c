/** @file
 * @brief 센서 보드의 메인 애플리케이션 파일
 *
 * 이 파일은 센서 읽기, BLE 광고 데이터 빌드 및 전송,
 * LED 상태 표시, 주기적인 Sleep/Wakeup 시퀀스를 관리합니다.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "gpio_if.h"
#include "dip_switch.h"
#include "ltc3337.h"
#include "sensors.h"
#include "ble_adv.h"
#include "debug.h"
#include "app_diag.h"
#include "sleep_if.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/**
 * @brief Manufacturer Specific Data를 담는 구조체 변수를 초기화합니다.
 */
static struct sensor_adv_data_t mfg_data = {
    // 0xFFFF를 Little Endian으로 변환하여 저장
    .company_id = sys_cpu_to_le16(0xFFFF), /* [Index 5-6] Company ID */
    .structure_version = 0x01,             /* [Index 7] Version v1 */
    .model_code = 0x01,                    /* [Index 8] Model Code (Inlet 압력) */

    // 상태 관리 정보
    .device_status = 0x00,       /* [Index 9] Device Status (정상) */
    .error_info = 0x00,          /* [Index 10] Error Info */
    .mcu_temperature = 25,       /* [Index 11] MCU Temperature (예: 25°C) */
    .battery_percent = 95,       /* [Index 12] Battery % (예: 95%) */
    .value_presence_mask = 0x01, /* [Index 13] Inlet 압력은 Sensor Value 1만 사용 (0b00000001) */

    // 센서 값 (압력 값 10.25 mmH2O를 100배하여 1025로 변환)
    .sensor_value_1 = sys_cpu_to_le32(1025), /* [Index 14-17] Sensor Value 1 (압력 값) */

    // 나머지 미사용 값은 0x00으로 설정됨
    .sensor_value_2 = 0, /* [Index 18-21] 미사용 */
    .sensor_value_3 = 0, /* [Index 22-25] 미사용 */
    .sensor_value_4 = 0, /* [Index 26-29] 미사용 */
    .sensor_value_5 = 0, /* [Index 30-33] 미사용 */
    .sensor_value_6 = 0, /* [Index 34-37] 미사용 */
};

/** @brief 메인 루프 작업을 위한 딜레이 가능 워크 */
static struct k_work_delayable loop_work;
/** @brief DIP 스위치 설정 값 */
static struct dip_bits g_dip;

/** @brief BLE 상태 값 */
static enum {
    BLE_STATE_NONE = 0,
    BLE_STATE_ADVING,
    BLE_STATE_SCAN_RSP,
} ble_state = BLE_STATE_NONE;

/* --------------------------------- LED Heartbeat --------------------------------- */

/** @brief LED Heartbeat 작업을 위한 딜레이 가능 워크 */
static struct k_work_delayable led_work;
/** @brief LED 켜짐/꺼짐 주기 (ms 단위) */
static uint32_t led_period_ms = 500;
/** @brief 현재 LED 상태 (켜짐/꺼짐) */
static bool led_state = false;

/**
 * @brief LED Heartbeat 함수. 주기적으로 LED 상태를 토글합니다.
 *
 * 광고 전송 상태에 따라 led_period_ms가 변경됩니다.
 * BLE 일시정지 상태일 경우 Heartbeat를 멈춥니다.
 *
 * @param w k_work 포인터 (사용되지 않음).
 */
static void led_fn(struct k_work *w)
{
    if (app_is_hold()) // BLE 일시정지 상태, 디버깅용
    {
        k_sleep(K_MSEC(50));
        return;
    }

    led_state = !led_state;
    board_led_set(led_state);
    k_work_schedule(&led_work, K_MSEC(led_period_ms));
}

/* --------------------------------- Sensor & Main Loop --------------------------------- */

/**
 * @brief 센서 값을 읽어 sensor_sample_t 구조체에 저장합니다.
 *
 * @param s 센서 값을 저장할 sensor_sample_t 구조체 포인터.
 */
static void get_sensor_data(sensor_sample_t *s)
{
    /* 1) 센서 읽기(간단) */
    read_pressure_0x28(s);
    int16_t t_cx100 = 0;
    read_ntc_ain1_cx100(&t_cx100);
    s->temperature_c_x100 = t_cx100;
    s->battery_pc = 100; // LTC3337 등을 통해 실제 배터리 잔량 업데이트 필요

    // MCU 온도 및 배터리 업데이트 (8-bit)
    Get_MCU_Temperature(&mfg_data.mcu_temperature);
    mfg_data.battery_percent++; // 예시용 임시 증가

    /* NOTE: device_status, error_info 필드는 여기서 오류 정보에 따라 업데이트 필요 */
    mfg_data.device_status = 0x00;
}

/**
 * @brief 메인 주기 루프 함수. 센서 데이터를 읽고 BLE 광고를 수행합니다.
 *
 * 이 함수는 k_work_delayable에 의해 주기적으로 호출됩니다.
 *
 * @param w k_work 포인터.
 */
static void loop_fn(struct k_work *w)
{
    int err = 0;
    Wakeup();

    /* 0) BLE 동작 테스트을 위한 디버그 코드 */

    // 버튼 입력 처리
    btn_evt_t btn = Get_BtnStatus();
    if (btn == BTN_EVT_LONG)
    {
        LOG_INF("Long button press detected");
        Init_Ble(BLE_SCAN_RESPONSE); /* 스캔 응답 설정 */
        ble_state = BLE_STATE_SCAN_RSP;
    }
    else if (btn == BTN_EVT_SHORT)
    {
        LOG_INF("Short button press detected");
        Init_Ble(BTN_ADV); /* 확장 광고 설정 */
        ble_state = BLE_STATE_ADVING;
    }
    else if (btn == BTN_EVT_NONE)
    {
        // 아무 동작 없음
    }

    // 스캔 응답은 다음 확장 광고 진행하지 않음
    if(ble_state == BLE_STATE_SCAN_RSP)
    {
        k_sleep(K_MSEC(50));
        goto SLEEP;
    }

    /* 1) 센서 읽기 */
    sensor_sample_t s = {0};
    get_sensor_data(&s);

    /* 2) Manufacturer 패킷 빌드 (정적 구조체 업데이트) */

    if (app_is_hold()) // BLE 정지, 디버깅용
    {
        k_sleep(K_MSEC(50));
        return;
    }

    /* 3) 광고 시작/갱신 (브로드캐스트 전용) */
    // mfg_data 구조체(33바이트)와 그 크기를 Tx_Ble에 전달
    err = Tx_Ble((const uint8_t *)&mfg_data, sizeof(mfg_data));

    /* 4) LED 속도(정상/에러) */
    // BLE 광고 성공 또는 진행 중일 경우 정상 속도(500ms), 오류 시 빠르게 깜빡임(120ms)
    bool ok = (err == 0) || (err == -EALREADY) || (err == -EINPROGRESS);
    led_period_ms = ok ? 500 : 120;

SLEEP:
    /* 5) 다음 주기 설정 (DIP 스위치 설정에 따라 10초 또는 5초) */
    uint32_t next_ms = g_dip.period ? 10000 : 5000;

    if (diag_on())
    {
        LOG_INF("loop: sleep=%ums, err=%d, P=%ld x100, T=%d x100, batt=%u%%, DIP{legacy-only, period=%u}",
                next_ms, err, (long)s.p_value_x100, s.temperature_c_x100, s.battery_pc, g_dip.period);
    }

    /* Sleep */
    Start_Sleep(next_ms / 1000);                  // 다음 스케쥴 관리로 sleep 시간 사용 안함
    k_work_schedule(&loop_work, K_MSEC(next_ms)); // 다음 루프 작업 예약
}

/**
 * @brief 메인 진입점 함수 (Entry Point)
 *
 * 시스템 초기화를 수행하고 주기적인 워크큐를 시작합니다.
 *
 * @return 항상 0.
 */
int main(void)
{
    printk("Sensor Board FW boot\n");
    board_gpio_init();

    /* DIP 스위치 읽기 */
    uint8_t raw = 0;
    dip_read_u8(&raw);
    g_dip = parse_dip(raw);

    /* 하드웨어 초기화 */
    sensors_init();
    ltc3337_init();

    /* BLE 초기화 */
    Init_Ble(BLE_INIT);
    ble_state = BLE_STATE_NONE;

    /* BLE 스캔 응답 모드로 설정 */
    // Init_Ble(BLE_SCAN_RESPONSE); /* 스캔 응답 설정 */
    // ble_state = BLE_STATE_SCAN_RSP;

    /* BLE 확장 광고로 설정 */
    Init_Ble(BTN_ADV); /* 확장 광고 설정 */
    ble_state = BLE_STATE_ADVING;

    /* 디버깅 코드 실행 */
    debug_run_startup();

    /* LED Heartbeat 작업 초기화 및 시작 */
    k_work_init_delayable(&led_work, led_fn);
    k_work_schedule(&led_work, K_MSEC(200));

    /* 메인 루프 작업 초기화 및 1초 후 시작 */
    k_work_init_delayable(&loop_work, loop_fn);
    k_work_schedule(&loop_work, K_SECONDS(1));

    return 0;
}