#include "lsm6dso.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <math.h>
#include <complex.h>
#include <string.h>

/**
 * @brief 원주율 (헤더에 정의되지 않은 경우 대비)
 * @note M_PI가 math.h에 없을 경우를 대비한 정의입니다.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(lsm6dso, LOG_LEVEL_INF);

/**
 * @brief Zephyr 스타일의 반환 코드(RC) 검사 매크로
 * @details
 * 표현식(expr)을 실행하고, 그 결과가 0 (성공)이 아니면 현재 함수에서 즉시 해당 오류 코드를 반환합니다.
 * @param expr 평가할 표현식 (주로 I2C R/W 함수 호출)
 */
#undef RC
#define RC(expr)                                                        \
    do                                                                  \
    {                                                                   \
        int __rc = (expr);                                              \
        if (__rc)                                                       \
        {                                                               \
            LOG_ERR("I2C Fail: %d at %s:%d", __rc, __func__, __LINE__); \
            return __rc;                                                \
        }                                                               \
    } while (0)

// I2C 반환 코드를 로그로 출력하는 매크로 (오류 발생 시 디버그 정보 추가)
#define LOG_RC(expr, name) ({                                                   \
    int __rc = (expr);                                                          \
    if (__rc != 0)                                                              \
    {                                                                           \
        LOG_ERR("I2C Error [%s]: %d at %s:%d", name, __rc, __func__, __LINE__); \
    }                                                                           \
    __rc;                                                                       \
})

#define RC_CHECK(rc) ({                                              \
    int __rc = (rc);                                                 \
    if (__rc != 0)                                                   \
    {                                                                \
        LOG_ERR("I2C Error: %d at %s:%d", __rc, __func__, __LINE__); \
        __rc = 1; /* 이 매크로 자체의 반환값 (성공 0, 실패 1) */     \
    }                                                                \
    else                                                             \
    {                                                                \
        __rc = 0;                                                    \
    }                                                                \
    __rc;                                                            \
})

/**
 * @name 전역 캡처 버퍼
 * @details
 * 쉘 스레드 등의 스택 오버플로우를 방지하기 위해 BSS 섹션에 큰 버퍼를 전역으로 할당합니다.
 * 이 버퍼들은 @ref lsm6dso_capture_once 및 FFT 처리(@ref bandlimited_rms_peak_ms2_x100)에서 사용됩니다.
 * @note 이름과 달리 실제로는 FIFO가 아닌 DRDY 폴링 캡처에 사용됩니다.
 * @{
 */
#define FIFO_WTM_WORDS 999 /**< 캡처할 최대 샘플 수 (워드) */

/* 캡처 대상 배열 (BSS) */
static int16_t g_ax[FIFO_WTM_WORDS]; /**< X축 가속도 LSB 데이터 버퍼 */
static int16_t g_ay[FIFO_WTM_WORDS]; /**< Y축 가속도 LSB 데이터 버퍼 */
static int16_t g_az[FIFO_WTM_WORDS]; /**< Z축 가속도 LSB 데이터 버퍼 */

/** @} */

/**
 * @name 하드웨어 및 I2C 정의
 * @{
 */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0)); /**< I2C0 디바이스 포인터 */
#define LSM6DSO_I2C_ADDR 0x6A                                         /**< LSM6DSO I2C 7비트 주소 */
/** @} */

/**
 * @name LSM6DSO 레지스터 주소
 * @{
 */
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_FIFO_CTRL5 0x0B // ODR_FIFO + FIFO_MODE
#define REG_INT1_CTRL 0x0D
#define REG_INT2_CTRL 0x0E
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL9_XL 0x18
#define REG_CTRL10_C 0x19
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_OUTX_L_A 0x28
#define REG_FIFO_DATA_OUT_TAG 0x78 /* 시작 주소(L) → 연속 버스트 읽기 */
#define REG_STATUS_REG 0x1E

/**
 * @brief I2C로 1바이트 쓰기
 * @param reg 대상 레지스터 주소
 * @param val 쓸 값
 * @return 0 on success, 음수 에러 코드 on failure.
 */
static int wr_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(i2c0, buf, sizeof(buf), LSM6DSO_I2C_ADDR);
}

/**
 * @brief I2C로 1바이트 읽기
 * @param reg 대상 레지스터 주소
 * @param[out] val 읽은 값을 저장할 포인터
 * @return 0 on success, 음수 에러 코드 on failure.
 */
static int rd_u8(uint8_t reg, uint8_t *val)
{
    return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, val, 1);
}

/**
 * @brief I2C로 블록 데이터 읽기
 * @param reg 시작 레지스터 주소
 * @param[out] buf 데이터를 저장할 버퍼
 * @param len 읽을 바이트 수
 * @return 0 on success, 음수 에러 코드 on failure.
 */
static int rd_block(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, buf, len);
}

/**
 * @brief 주요 레지스터 값을 Zephyr 쉘에 덤프합니다.
 * @param shell 쉘 인스턴스 포인터
 * @return 0 on success, 음수 에러 코드 on failure (I2C 오류 발생 시).
 */
int lsm6dso_dump_regs(const struct shell *shell)
{
    uint8_t v;
    uint8_t b[2];

    rd_u8(REG_WHO_AM_I, &v);
    shell_print(shell, "WHO_AM_I      : 0x%02X", v);
    rd_u8(REG_CTRL1_XL, &v);
    shell_print(shell, "CTRL1_XL      : 0x%02X", v);
    rd_u8(REG_CTRL2_G, &v);
    shell_print(shell, "CTRL2_G       : 0x%02X", v);
    rd_u8(REG_CTRL3_C, &v);
    shell_print(shell, "CTRL3_C       : 0x%02X", v);
    rd_u8(REG_CTRL9_XL, &v);
    shell_print(shell, "CTRL9_XL      : 0x%02X", v);
    rd_u8(REG_FIFO_CTRL3, &v);
    shell_print(shell, "FIFO_CTRL3    : 0x%02X", v);
    rd_u8(REG_FIFO_CTRL4, &v);
    shell_print(shell, "FIFO_CTRL4    : 0x%02X", v);
    rd_u8(REG_FIFO_CTRL5, &v);
    shell_print(shell, "FIFO_CTRL5    : 0x%02X", v);

    rd_block(REG_FIFO_STATUS1, b, 2);
    uint16_t diff = ((uint16_t)(b[1] & 0x0F) << 8) | b[0];
    shell_print(shell, "FIFO_STATUS1/2: %02X %02X  (DIFF=%u)", b[0], b[1], diff);

    return 0;
}

/*
 ==============================================================================
 * 전역 변수 및 상수 정의
 ==============================================================================
 */

/**
 * @brief 전역 진동 데이터 결과 인스턴스
 */
vibration_results_t g_vib_results;

/**
 * @brief FIFO 버스트 읽기를 위한 원시(Raw) 버퍼
 * @details 999 샘플 * 7 바이트/샘플 (Tag + XYZ) = 6993 바이트 [cite: 28]
 */
static uint8_t g_fifo_raw_buf[6993];

/**
 * @name 진동 처리 상수 (요구사항 기반)
 * @{
 */
#define VIB_N 999                          /**< 샘플 수 (N)  */
#define VIB_FS 3330.0f                     /**< 샘플링 주파수 (fs) (3.33kHz ODR) [cite: 15, 39] */
#define VIB_DF (VIB_FS / VIB_N)            /**< 주파수 해상도 (Delta f) (10/3 Hz) [cite: 39] */
#define VIB_K_MIN 3                        /**< 최소 주파수 인덱스 (10 Hz) [cite: 40] */
#define VIB_K_MAX 300                      /**< 최대 주파수 인덱스 (1000 Hz) [cite: 40] */
#define VIB_CG 0.4994994995f               /**< Hann 윈도우 진폭 보정 계수 (CG) [cite: 41, 77] */
#define VIB_U 0.3746246246f                /**< Hann 윈도우 전력 보정 계수 (U) [cite: 41] */
#define VIB_SA_4G 0.001196f                /**< ±4g 스케일 팩터 (m/s^2)/LSB [cite: 43] */
#define VIB_MS2_TO_MG (1.0f / 0.00980665f) /**< m/s^2 를 mg 로 변환 [cite: 66, 75] */
#define VIB_M_TO_MM 1000.0f                /**< m 를 mm 로 변환 [cite: 114, 120] */
/** @} */

/**
 * @name DSP 처리용 전역 버퍼
 * @details
 * 스택 오버플로우를 방지하기 위해 BSS 섹션에 정적 할당합니다.
 * 3축이 순차적으로 이 버퍼를 재사용합니다.
 * @{
 */
/** @brief 전처리(스케일링, DC제거, 윈도우) 후 FFT 입력 버퍼 */
static float g_dsp_in_buf[VIB_N];
/** @brief rFFT 출력 버퍼 (복소수) (N=999일 때 (999+1)/2 = 500개) */
static float complex g_dsp_X_k[(VIB_N + 1) / 2];
/** @brief 가속도 Peak용 IFFT 입력 버퍼 (대역 필터링됨) */
static float complex g_dsp_A_k[(VIB_N + 1) / 2];
/** @brief 속도 Peak용 IFFT 입력 버퍼 (대역 필터링 및 적분됨) */
static float complex g_dsp_Bv_k[(VIB_N + 1) / 2];
/** @brief 가속도 Peak 계산용 IFFT 출력 버퍼 */
static float g_dsp_a_bp[VIB_N];
/** @brief 속도 Peak 계산용 IFFT 출력 버퍼 */
static float g_dsp_v_bp[VIB_N];
/** @} */

/*
 ==============================================================================
 * 중요: FFT/IFFT 스텁 (Stub) 함수
 ==============================================================================
 */

/**
 * @brief N=999 길이의 실수 FFT (Real FFT) 스텁
 * @warning
 * 이것은 실제 구현이 아닙니다!
 * nRF Connect SDK의 표준 CMSIS-DSP 라이브러리는 N=999를 지원하지 않습니다.
 * N=999를 처리할 수 있는 별도의 FFT 구현 (예: Slow DFT)으로
 * 이 함수를 반드시 교체해야 합니다.
 *
 * @param in 실수 입력 배열 (크기 VIB_N)
 * @param out 복소수 출력 배열 (크기 (VIB_N+1)/2 = 500)
 */
static void RFFT_N999(float *in, float complex *out)
{
    /*
     * !!! 중요: 실제 N=999 FFT 구현으로 이 코드를 교체하십시오 !!!
     *
     * 예:
     * my_custom_rfft_f32(in, out, VIB_N);
     *
     * (참고: CMSIS-DSP의 arm_rfft_fast_f32는 N=1024만 지원합니다)
     */
    LOG_WRN("RFFT_N999() is a STUB! Replace with actual N=999 FFT implementation.");
    // 스텁: 출력을 0으로 초기화
    memset(out, 0, sizeof(g_dsp_X_k));
}

/**
 * @brief N=999 길이의 실수 IFFT (Inverse Real FFT) 스텁
 * @warning
 * 이것은 실제 구현이 아닙니다!
 * RFFT_N999()와 쌍을 이루는 실제 IFFT 구현으로 이 함수를
 * 반드시 교체해야 합니다.
 *
 * @param in 복소수 입력 배열 (크기 (VIB_N+1)/2 = 500)
 * @param out 실수 출력 배열 (크기 VIB_N)
 */
static void IRFFT_N999(float complex *in, float *out)
{
    /*
     * !!! 중요: 실제 N=999 IFFT 구현으로 이 코드를 교체하십시오 !!!
     *
     * 예:
     * my_custom_irfft_f32(in, out, VIB_N);
     */
    LOG_WRN("IRFFT_N999() is a STUB! Replace with actual N=999 IFFT implementation.");
    // 스텁: 출력을 0으로 초기화
    memset(out, 0, VIB_N * sizeof(float));
}

/*
 ==============================================================================
 * 센서 제어 함수
 ==============================================================================
 */

/* lsm6dso.c 파일 내 lsm6dso_init 함수 수정 */
int lsm6dso_init(void)
{
    uint8_t who_am_i;
    RC(rd_u8(REG_WHO_AM_I, &who_am_i));

    if (who_am_i != 0x6C) { // LSM6DSO Who Am I
        LOG_ERR("LSM6DSO Who Am I check FAILED. Got 0x%02X", who_am_i);
        return -ENODEV;
    }
    LOG_INF("LSM6DSO Who Am I check PASSED. (0x%02X)", who_am_i);

    /* --- [추가] 소프트 리셋 및 재부팅 대기 --- */
    LOG_INF("Performing soft reset...");
    RC(wr_u8(REG_CTRL3_C, 0x80)); // SW_RESET=1 (bit 7)
    k_msleep(10); // 부팅 대기
    
    // BDU=1, IF_INC=1
    RC(wr_u8(REG_CTRL3_C, 0x40 | 0x04)); // 0x44 (BDU=1, IF_INC=1)
    
    /* --- 부팅 및 센서 설정 (Sensor 6/7 요구사항) --- */
    // CTRL9_XL: I3C_disable=1
    RC(wr_u8(REG_CTRL9_XL, 0x02));
    // CTRL1_XL: ODR_XL=3.33 kHz, FS_XL=±4g
    RC(wr_u8(REG_CTRL1_XL, 0x90)); // ODR=1001b, FS=00b
    // CTRL2_G: ODR_G=Power-down (자이로 OFF)
    RC(wr_u8(REG_CTRL2_G, 0x00));

    /* --- FIFO 설정 (Sensor 6/7 요구사항) --- */
    // FIFO_CTRL3: BDR_XL=3.33 kHz
    RC(wr_u8(REG_FIFO_CTRL3, 0x09)); // BDR_XL=1001b
    // FIFO_CTRL1/2: WTM = 999 워드 (0x3E7)
    RC(wr_u8(REG_FIFO_CTRL1, 0xE7)); // WTM_L (999 & 0xFF)
    RC(wr_u8(REG_FIFO_CTRL2, 0x03)); // WTM_H (999 >> 8)
    
    // [추가] FIFO_CTRL5: ODR_FIFO=3.33kHz (센서 7 요구사항 반영)
    // ODR_FIFO=1001b (bit 2:4) + FIFO_MODE=000b (bit 0:2, 나중에 리셋에서 변경)
    RC(wr_u8(REG_FIFO_CTRL5, 0x90)); // ODR_FIFO = 3.33k
    
    // FIFO_CTRL4: FIFO_MODE=Bypass(000b), STOP_ON_WTM=1
    RC(wr_u8(REG_FIFO_CTRL4, 0x01)); // 초기 상태는 Bypass로 설정
    
    LOG_INF("LSM6DSO initialized (ODR=3.33k, FIFO_WTM=999)");
    return 0;
}

/**
 * @brief FIFO를 리셋합니다. (Bypass 모드 -> FIFO 모드) [cite: 22]
 */
/* lsm6dso_fifo_reset 함수 수정 */
static int lsm6dso_fifo_reset(void)
{
    uint32_t t_start = k_uptime_get_32();
    int rc;

    // 1. FIFO_MODE=Bypass(000b), STOP_ON_WTM=1 유지 (FIFO 비활성화)
    //    -> 이 시점에서 FIFO는 비워짐 (클리어)
    rc = wr_u8(REG_FIFO_CTRL4, 0x01);
    LOG_DBG("FIFO_CTRL4->0x01 (Bypass): rc=%d, t=%u", rc, (unsigned)(k_uptime_get_32() - t_start));
    if (rc) return rc;
    
    k_msleep(2); // FIFO 클리어 및 모드 전환 대기

    // 2. FIFO_MODE=FIFO(001b), STOP_ON_WTM=1 유지 (캡처 시작)
    rc = wr_u8(REG_FIFO_CTRL4, 0x03);
    LOG_DBG("FIFO_CTRL4->0x03 (FIFO Mode): rc=%d, t=%u", rc, (unsigned)(k_uptime_get_32() - t_start));
    if (rc) return rc;

    return 0;
}

/**
 * @brief FIFO가 찰 때까지 폴링한 후, 999개 샘플을 버스트로 읽어옵니다.
 *
 * @return 0 on success, 음수 에러 코드 on failure.
 */
/* lsm6dso_read_fifo_burst 함수 수정 */
static int lsm6dso_read_fifo_burst(void)
{
    uint8_t b[2]; // STATUS1, 2 레지스터 값을 받을 버퍼
    uint16_t diff = 0;
    int retries = 200; // 1초 타임아웃으로 증가
    uint32_t t_start = k_uptime_get_32();
    uint8_t ctrl1_xl, fifo_ctrl4, who_am_i;

    // [디버그] 초기 상태 및 설정 값 확인 (오류 직전 상태 파악)
    // FIFO_CTRL4 (모드)와 CTRL1_XL (ODR/FS)이 핵심
    LOG_RC(rd_u8(REG_CTRL1_XL, &ctrl1_xl), "CTRL1_XL");
    LOG_RC(rd_u8(REG_FIFO_CTRL4, &fifo_ctrl4), "FIFO_CTRL4");
    LOG_RC(rd_u8(REG_WHO_AM_I, &who_am_i), "WHO_AM_I");
    
    // (rc가 -116인 경우도 많으므로, LOG_ERR 대신 LOG_DBG 사용)
    LOG_DBG("Poll start check: WHO=0x%02X, XL=0x%02X, FIFO_C4=0x%02X", 
            who_am_i, ctrl1_xl, fifo_ctrl4);

    // 1. 워터마크 확인 (폴링) - DIFF_FIFO 개수 직접 확인
    do
    {
        int rc;

        // STATUS1(0x3A), STATUS2(0x3B) 블록 읽기
        rc = rd_block(REG_FIFO_STATUS1, b, 2);
        if (rc)
        {
            LOG_ERR("I2C Fail on FIFO_STATUS: %d", rc);
            return rc;
        }

        diff = ((uint16_t)(b[1] & 0x0F) << 8) | b[0];
        uint8_t status2_val = b[1]; // FIFO_STATUS2 값

        // [디버그] 폴링 중간 상태 로깅 (50ms마다, 10회마다)
        if ((200 - retries) % 10 == 0)
        {
            LOG_DBG("t=%u, DIFF=%u, STAT1=0x%02X, STAT2=0x%02X (WTM=%d, FULL=%d, OVR=%d)",
                    (unsigned)(k_uptime_get_32() - t_start),
                    diff, b[0], b[1],
                    (status2_val >> 7) & 0x01,  // WTM_IA
                    (status2_val >> 5) & 0x01,  // FIFO_FULL_IA
                    (status2_val >> 6) & 0x01); // OVR_LA
        }

        // 999개 이상 샘플이 모였으면 루프 탈출
        if (diff >= FIFO_WTM_WORDS)
        {
            break;
        }

        k_msleep(5); // 5ms 대기
    } while (--retries > 0);

    if (retries == 0)
    {
        LOG_ERR("FIFO sample count TIMEOUT! (Diff=%u, Expected %u) after %u ms. RC=-116",
                diff, FIFO_WTM_WORDS, (unsigned)(k_uptime_get_32() - t_start));
        return -ETIMEDOUT; // -ETIMEDOUT은 Zephyr의 -116
    }

    // WTM이 아닌 오버플로우로 채워진 경우 (경고 로그는 유지)
    if (diff != FIFO_WTM_WORDS)
    {
        LOG_WRN("FIFO WTM reached, but count is %u (expected %u). Proceeding.", diff, FIFO_WTM_WORDS);
    }

    // 2. 단일 버스트 읽기 (6993 바이트)
    // [디버그] 버스트 읽기 직전 로그
    uint32_t t_burst_start = k_uptime_get_32();
    LOG_DBG("FIFO count reached. Starting %zu byte burst read...", sizeof(g_fifo_raw_buf));

    // 버스트 읽기
    int rc = rd_block(REG_FIFO_DATA_OUT_TAG, g_fifo_raw_buf, sizeof(g_fifo_raw_buf));
    if (rc)
    {
        LOG_ERR("I2C Fail on FIFO burst read: %d", rc);
        return rc;
    }

    // [디버그] 버스트 읽기 후 로그
    LOG_DBG("Burst read complete in %u ms.", (unsigned)(k_uptime_get_32() - t_burst_start));

    // 3. 원시 버퍼를 3축 (g_ax, g_ay, g_az)으로 파싱
    // (이하 동일)
    for (int i = 0; i < VIB_N; i++)
    {
        size_t offset = i * 7;
        // uint8_t tag = g_fifo_raw_buf[offset]; // (필요시) tag(0x01) 검사
        g_ax[i] = (int16_t)((g_fifo_raw_buf[offset + 2] << 8) | g_fifo_raw_buf[offset + 1]);
        g_ay[i] = (int16_t)((g_fifo_raw_buf[offset + 4] << 8) | g_fifo_raw_buf[offset + 3]);
        g_az[i] = (int16_t)((g_fifo_raw_buf[offset + 6] << 8) | g_fifo_raw_buf[offset + 5]);
    }

    return 0;
}

/*
 ==============================================================================
 * 진동 처리 (DSP) 함수
 ==============================================================================
 */

/**
 * @brief 단일 축에 대해 가속도/속도 RMS 및 Peak를 계산합니다.
 *
 * @param raw_data [in] 999개의 LSB 샘플 배열 (g_ax, g_ay, g_az 중 하나)
 * @param axis_index [in] 결과를 저장할 축 인덱스 (0=X, 1=Y, 2=Z)
 */
static void lsm6dso_process_axis(int16_t *raw_data, int axis_index)
{
    /* --- 1, 2. 입력/전처리 (스케일링, DC 제거) [cite: 47, 48] --- */
    double mean = 0.0;
    for (int n = 0; n < VIB_N; n++)
    {
        mean += (double)raw_data[n];
    }
    mean /= VIB_N;
    mean *= VIB_SA_4G; // LSB 평균 -> (m/s^2) 평균

    /* --- 3. Hann 윈도우 적용 [cite: 49, 50] --- */
    for (int n = 0; n < VIB_N; n++)
    {
        float a_n = (float)raw_data[n] * VIB_SA_4G;                      // a[n]
        float a0_n = a_n - (float)mean;                                  // a0[n] (DC 제거)
        float w_n = 0.5f * (1.0f - cosf(2.0f * M_PI * n / (VIB_N - 1))); // w[n]
        g_dsp_in_buf[n] = a0_n * w_n;                                    // x[n]
    }

    /* --- 4. FFT (rFFT) [cite: 52, 98] --- */
    // g_dsp_in_buf (N=999) -> g_dsp_X_k (N=500, complex)
    RFFT_N999(g_dsp_in_buf, g_dsp_X_k);

    float a_rms_sum = 0.0f;
    float v_rms_sum = 0.0f;
    const float U_FS_N = VIB_U * VIB_FS * VIB_N;

    // g_dsp_A_k, g_dsp_Bv_k 0으로 초기화
    memset(g_dsp_A_k, 0, sizeof(g_dsp_A_k));
    memset(g_dsp_Bv_k, 0, sizeof(g_dsp_Bv_k));

    /* --- 5, 6, 7, 8 (가속도/속도 스펙트럼, PSD, RMS/Peak 동시 처리) --- */
    // k=0 ~ (N-1)/2 (즉, 0 ~ 499) 까지 순회
    for (int k = 0; k <= (VIB_N - 1) / 2; k++)
    {
        // g_dsp_X_k[k] 는 X[k] (복소수)
        float mag_Xk_sq = cpowf(cabsf(g_dsp_X_k[k]), 2);
        float Paa_k;

        // 5. 가속도 PSD (Paa) [cite: 53, 54]
        if (k == 0)
        {
            Paa_k = mag_Xk_sq / U_FS_N;
        }
        else
        {
            Paa_k = 2.0f * mag_Xk_sq / U_FS_N;
        }

        // 6, 7, 10, 11. 밴드 마스크 (10-1000Hz, 즉 k=3..300) [cite: 40, 61, 108]
        if (k >= VIB_K_MIN && k <= VIB_K_MAX)
        {
            /* === 가속도 처리 === */
            // 7. 가속도 RMS (대역 한정) [cite: 63, 64]
            a_rms_sum += (Paa_k * VIB_DF);

            // 8. 가속도 Peak용 스펙트럼 (A[k]) 저장 [cite: 69, 70]
            g_dsp_A_k[k] = g_dsp_X_k[k];

            /* === 속도 처리 === */
            // 5. (속도) 주파수 적분 (V[k]) [cite: 100, 103]
            float f_k = (float)k * VIB_DF;
            // V[k] = X[k] / (j * 2 * pi * f_k)
            float complex Vk = g_dsp_X_k[k] / ((0.0f + 1.0f * I) * 2.0f * M_PI * f_k);

            // 6. (속도) PSD (Pvv) [cite: 105]
            float mag_Vk_sq = cpowf(cabsf(Vk), 2);
            float Pvv_k = 2.0f * mag_Vk_sq / U_FS_N; // (k>=3 이므로 k=0 경우는 무시)

            // 8. (속도) RMS (대역 한정) [cite: 111, 112]
            v_rms_sum += (Pvv_k * VIB_DF);

            // 9. (속도) Peak용 스펙트럼 (Bv[k]) 저장 [cite: 117]
            g_dsp_Bv_k[k] = Vk;
        }
    } // end for k

    /* --- 가속도 RMS 최종 계산 --- */
    float a_rms = sqrtf(a_rms_sum);
    g_vib_results.a_rms_mg[axis_index] = a_rms * VIB_MS2_TO_MG;

    /* --- 속도 RMS 최종 계산 --- */
    float v_rms = sqrtf(v_rms_sum);
    g_vib_results.v_rms_mmps[axis_index] = v_rms * VIB_M_TO_MM;

    /* --- 9, 10. 가속도 Peak (대역 한정) --- */
    IRFFT_N999(g_dsp_A_k, g_dsp_a_bp); // 9. a_bp[n] = IFFT(A[k]) [cite: 72]
    float a_peak = 0.0f;
    for (int n = 0; n < VIB_N; n++)
    {
        float abs_val = fabsf(g_dsp_a_bp[n]);
        if (abs_val > a_peak)
        {
            a_peak = abs_val;
        }
    }
    a_peak /= VIB_CG; // 10. a_Peak = max(|a_bp[n]|) / CG [cite: 74]
    g_vib_results.a_peak_mg[axis_index] = a_peak * VIB_MS2_TO_MG;

    /* --- 10, 11. 속도 Peak (대역 한정) --- */
    IRFFT_N999(g_dsp_Bv_k, g_dsp_v_bp); // 10. v_bp[n] = IFFT(Bv[k]) [cite: 118]
    float v_peak = 0.0f;
    for (int n = 0; n < VIB_N; n++)
    {
        float abs_val = fabsf(g_dsp_v_bp[n]);
        if (abs_val > v_peak)
        {
            v_peak = abs_val;
        }
    }
    v_peak /= VIB_CG; // 11. v_Peak = max(|v_bp[n]|) / CG [cite: 119, 121]
    g_vib_results.v_peak_mmps[axis_index] = v_peak * VIB_M_TO_MM;
}

int lsm6dso_capture_and_process(void)
{
    LOG_INF("Starting capture & process (N=%d)...", VIB_N);

    // 1. FIFO 리셋
    RC(lsm6dso_fifo_reset());

    // 2. FIFO 채움 (MCU 슬립)
    // N=999, fs=3330Hz -> 999 / 3330 = 0.3s
    // 여유를 두어 310ms 슬립
    /* k_msleep(310);  <--- 이 줄을 주석 처리하거나 삭제합니다! */

    // 3. FIFO 버스트 읽기 및 파싱
    //    (내부의 폴링 루프가 k_msleep(5)를 호출하므로
    //     이 함수 자체가 슬립/대기 역할을 합니다.)
    LOG_DBG("Waiting for FIFO WTM...");
    int rc = lsm6dso_read_fifo_burst();
    if (rc)
    {
        LOG_ERR("Failed to read FIFO burst: %d", rc);
        return rc;
    }
    LOG_INF("FIFO read complete. Processing 3 axes...");

    // 4. 축별 처리
    lsm6dso_process_axis(g_ax, 0); // X-axis
    LOG_DBG("X-axis processed.");
    lsm6dso_process_axis(g_ay, 1); // Y-axis
    LOG_DBG("Y-axis processed.");
    lsm6dso_process_axis(g_az, 2); // Z-axis
    LOG_DBG("Z-axis processed.");

    LOG_INF("Processing complete.");
    LOG_INF("--- Results ---");
    LOG_INF("A-RMS (mg): X=%.2f, Y=%.2f, Z=%.2f",
            g_vib_results.a_rms_mg[0], g_vib_results.a_rms_mg[1], g_vib_results.a_rms_mg[2]);
    LOG_INF("A-Peak(mg): X=%.2f, Y=%.2f, Z=%.2f",
            g_vib_results.a_peak_mg[0], g_vib_results.a_peak_mg[1], g_vib_results.a_peak_mg[2]);
    LOG_INF("V-RMS (mm/s): X=%.2f, Y=%.2f, Z=%.2f",
            g_vib_results.v_rms_mmps[0], g_vib_results.v_rms_mmps[1], g_vib_results.v_rms_mmps[2]);
    LOG_INF("V-Peak(mm/s): X=%.2f, Y=%.2f, Z=%.2f",
            g_vib_results.v_peak_mmps[0], g_vib_results.v_peak_mmps[1], g_vib_results.v_peak_mmps[2]);

    return 0;
}

/**
 * @brief 1회성 FIFO 캡처 및 가속도 통계(전체/대역) 계산
 *
 * 이 함수는 N=999 샘플을 캡처하여 lsm6dso_stats_t 구조체를 채웁니다.
 * 1. FIFO를 리셋하고 N=999 샘플(약 300ms)을 캡처합니다.
 * 2. 캡처된 원시 데이터(g_ax, g_ay, g_az)를 사용하여 전체 대역(Full-Band)의
 * 시간-도메인 RMS 및 Peak를 계산합니다. (DC 오프셋 제거 포함)
 * 3. 이전에 구현된 lsm6dso_process_axis()를 호출하여 10-1000Hz 대역 제한(Band-Limited)
 * 주파수-도메인 RMS 및 Peak를 계산합니다. (g_vib_results 전역 변수 사용)
 * 4. 모든 결과를 m/s^2 * 100 스케일의 int16_t 고정소수점으로 변환하여 'out' 구조체에 저장합니다.
 *
 * @param out [out] 통계 결과를 저장할 lsm6dso_stats_t 구조체 포인터
 * @return 0 on success, 음수 에러 코드 on failure.
 */
int lsm6dso_capture_once(lsm6dso_stats_t *out)
{
    // 0. 출력 구조체 초기화
    memset(out, 0, sizeof(lsm6dso_stats_t));

    // 1. FIFO 리셋
    RC(lsm6dso_fifo_reset());

    // 2. FIFO 채움 대기 (MCU 슬립)
    // N=999, fs=3330Hz -> 999 / 3330 = 0.3s
    k_msleep(310); // 여유 시간 포함

    // 3. FIFO 버스트 읽기 및 파싱
    LOG_DBG("FIFO full, reading burst...");
    int rc = lsm6dso_read_fifo_burst();

    // 4. 디버그 정보 채우기
    rd_u8(REG_WHO_AM_I, &out->whoami);
    out->n = VIB_N;

    if (rc)
    {
        LOG_ERR("Failed to read FIFO burst: %d", rc);
        out->wtm_reached = false;
        return rc;
    }
    out->wtm_reached = true;
    LOG_INF("FIFO read complete. Processing stats...");

    // 5. 전체 대역 (Full-Band) 시간-도메인 통계 계산
    int16_t *axes_buf[3] = {g_ax, g_ay, g_az};
    for (int i = 0; i < 3; i++)
    {
        int16_t *raw = axes_buf[i];

        // 5.1. DC 오프셋 (평균) 계산 (LSB)
        double mean_lsb = 0.0;
        for (int n = 0; n < VIB_N; n++)
        {
            mean_lsb += (double)raw[n];
        }
        mean_lsb /= VIB_N;

        // 5.2. DC 제거된 값으로 RMS(제곱합), Peak(절대값) 계산
        double rms_sum_sq_lsb = 0.0;
        int32_t peak_lsb = 0; // int16_t의 절대값을 다루므로 32비트 사용

        for (int n = 0; n < VIB_N; n++)
        {
            double val_dc_removed_lsb = (double)raw[n] - mean_lsb;
            int32_t abs_val = (int32_t)fabs(val_dc_removed_lsb);

            if (abs_val > peak_lsb)
            {
                peak_lsb = abs_val;
            }
            rms_sum_sq_lsb += (val_dc_removed_lsb * val_dc_removed_lsb);
        }

        // 5.3. RMS 및 Peak (LSB) -> (m/s^2) 변환
        double rms_lsb = sqrt(rms_sum_sq_lsb / VIB_N);
        float peak_ms2 = (float)peak_lsb * VIB_SA_4G;
        float rms_ms2 = (float)rms_lsb * VIB_SA_4G;

        // 5.4. (m/s^2 * 100) 고정소수점으로 변환하여 저장
        out->peak_ms2_x100[i] = (int16_t)roundf(peak_ms2 * 100.0f);
        out->rms_ms2_x100[i] = (int16_t)roundf(rms_ms2 * 100.0f);
    }
    LOG_DBG("Full-band time-domain stats calculated.");

    // 6. 대역 제한 (Band-Limited) 주파수-도메인 통계 계산
    //    (이전에 구현된 lsm6dso_process_axis 함수 재사용)
    lsm6dso_process_axis(g_ax, 0); // X-axis
    lsm6dso_process_axis(g_ay, 1); // Y-axis
    lsm6dso_process_axis(g_az, 2); // Z-axis

    // 6.1. g_vib_results (float mg) -> (int16_t m/s^2 * 100) 변환
    for (int i = 0; i < 3; i++)
    {
        // g_vib_results.a_rms_mg[i] 는 [mg] 단위
        // [mg] / VIB_MS2_TO_MG = [m/s^2]
        float a_rms_ms2 = g_vib_results.a_rms_mg[i] / VIB_MS2_TO_MG;
        float a_peak_ms2 = g_vib_results.a_peak_mg[i] / VIB_MS2_TO_MG;

        // 6.2. (m/s^2 * 100) 고정소수점으로 변환하여 저장
        out->bl_rms_ms2_x100[i] = (int16_t)roundf(a_rms_ms2 * 100.0f);
        out->bl_peak_ms2_x100[i] = (int16_t)roundf(a_peak_ms2 * 100.0f);
    }
    LOG_DBG("Band-limited freq-domain stats calculated.");
    LOG_INF("lsm6dso_capture_once complete.");

    return 0;
}
