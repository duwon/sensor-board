/*
 * lsm6dso.h
 *
 * LSM6DSO 6축 IMU 센서 드라이버 (FIFO 및 진동 처리 특화)
 */
#ifndef LSM6DSO_H_
#define LSM6DSO_H_

#include <zephyr/shell/shell.h>
#include <stdbool.h>

/**
 * @brief 진동 데이터 (가속도/속도) RMS 및 Peak 결과
 */
typedef struct
{
    /** @brief 3축 가속도 RMS [mg] (X, Y, Z) */
    float a_rms_mg[3];
    /** @brief 3축 가속도 Peak [mg] (X, Y, Z) */
    float a_peak_mg[3];
    /** @brief 3축 속도 RMS [mm/s] (X, Y, Z) */
    float v_rms_mmps[3];
    /** @brief 3축 속도 Peak [mm/s] (X, Y, Z) */
    float v_peak_mmps[3];
} vibration_results_t;
typedef struct
{
    uint16_t n; /* 수집된 샘플 수 */

    /* 전체대역 통계 (m/s^2 ×100 고정소수) */
    int16_t peak_ms2_x100[3];
    int16_t rms_ms2_x100[3];

    /* 10–1000 Hz 대역 제한 통계 (m/s^2 ×100) */
    int16_t bl_peak_ms2_x100[3];
    int16_t bl_rms_ms2_x100[3];

    /* 디버그 */
    uint8_t whoami;
    bool wtm_reached;
} lsm6dso_stats_t;

/**
 * @brief 전역 진동 데이터 결과
 * @details
 * 3축(X, Y, Z)에 대한 가속도/속도의 RMS/Peak 값을 저장합니다.
 * 이 구조체는 lsm6dso_capture_and_process()에 의해 업데이트됩니다.
 */
extern vibration_results_t g_vib_results;

/**
 * @brief LSM6DSO 센서를 초기화합니다.
 *
 * 요구사항에 명시된 레지스터 값으로 센서를 설정합니다.
 * (ODR=3.33kHz, FS=±4g, Gyro=OFF, FIFO=999, BDU=1, IF_INC=1 등)
 *
 * @return 0 on success, 음수 에러 코드 on failure.
 */
int lsm6dso_init(void);

/**
 * @brief 999개의 샘플을 캡처하고 가속도/속도 처리를 수행합니다.
 *
 * 1. FIFO 리셋 [cite: 22]
 * 2. FIFO가 찰 때까지 대기 (약 300ms) [cite: 23, 25]
 * 3. FIFO 데이터를 버스트 모드로 읽기 [cite: 27, 28]
 * 4. 3축(X, Y, Z) 데이터 파싱
 * 5. 각 축에 대해 가속도/속도 DSP 처리 (FFT, PSD, RMS/Peak)
 * 6. 결과를 g_vib_results 구조체에 업데이트
 *
 * @return 0 on success, 음수 에러 코드 on failure.
 */
int lsm6dso_capture_and_process(void);

struct shell;

/**
 * @brief 주요 레지스터 값을 Zephyr 쉘에 덤프합니다. (제공된 함수)
 * @param shell 쉘 인스턴스 포인터
 * @return 0 on success, 음수 에러 코드 on failure.
 */
int lsm6dso_dump_regs(const struct shell *shell);

#endif /* LSM6DSO_H_ */