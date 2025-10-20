/**
 * @file lsm6dso.c
 * @brief LSM6DSO IMU 드라이버 (Zephyr)
 *
 * @details
 * 이 드라이버는 LSM6DSO 가속도계 센서에 중점을 둡니다.
 * 3.33kHz의 고속 샘플링(ODR) 설정 후, FIFO를 사용하지 않고 DRDY(Data Ready) 비트를 폴링(polling)하여 데이터를 직접 캡처합니다.
 *
 * 캡처된 데이터는 내장된 1024-point FFT 루틴을 통해 전체 대역(Broadband) 및 특정 대역(10-1000Hz)의 RMS 및 Peak 값을 m/s^2 단위로 계산하는 데 사용됩니다.
 */

#include "lsm6dso.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <math.h>
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
#define RC(expr)           \
    do                     \
    {                      \
        int __rc = (expr); \
        if (__rc)          \
            return __rc;   \
    } while (0)

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
#define REG_FIFO_CTRL5 0x0B
#define REG_INT1_CTRL 0x0D
#define REG_INT2_CTRL 0x0E
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL9_XL 0x18
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_OUTX_L_A 0x28
#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_STATUS_REG 0x1E
/** @} */

#define WHOAMI_EXPECTED 0x6C /**< WHO_AM_I 레지스터 기대값 */

/**
 * @name 레지스터 비트필드 정의
 * @{
 */

/* REG_CTRL3_C */
#define CTRL3_C_BDU BIT(6)    /**< Block Data Update */
#define CTRL3_C_IF_INC BIT(2) /**< Register address auto-increment */

/* REG_CTRL9_XL */
#define CTRL9_XL_I3C_DISABLE BIT(1) /**< I3C 인터페이스 비활성화 */

/* REG_CTRL1_XL: ODR, FS */
/**
 * @brief 가속도 ODR: 3.33 kHz (1010b)
 * @note @ref lsm6dso_init 에서는 0x9 (3.33kHz)를 사용합니다.
 */
#define ODR_XL_3k33 (0x0A << 4)
#define FS_XL_4G (0x2 << 2)  /**< 가속도 Full-Scale: ±4g (10b) */
#define FS_XL_16G (0x1 << 2) /**< 가속도 Full-Scale: ±16g (01b) */

/* REG_CTRL2_G: Gyroscope */
#define ODR_G_POWER_DOWN (0x0 << 4) /**< 자이로스코프 파워 다운 */

/* FIFO 관련 설정 */
#define ODR_FIFO_3k33 0x0A /**< FIFO ODR: 3.33 kHz (1010b) */
/**
 * @brief FIFO 가속도 BDR: 3.33 kHz (1010b)
 * @note @ref lsm6dso_init 에서는 0x9 (3.33kHz)를 사용합니다.
 */
#define BDR_XL_3k33 (0x0A)
#define FIFO_CTRL4_STOP_ON_WTM BIT(5) /**< Watermark 도달 시 FIFO 중지 (0=Overwrite) */
#define FIFO_MODE_BYPASS 0x0          /**< FIFO 모드: Bypass (000b) */
#define FIFO_MODE_FIFO 0x1            /**< FIFO 모드: FIFO (001b) */
#define FIFO_MODE_CONTINUOUS 0x6      /**< FIFO 모드: Continuous (110b) */

/* REG_STATUS_REG */
#define XLDA_BIT BIT(0) /**< Accelerometer new data available */
/** @} */

/**
 * @name 샘플링 및 FFT 정의
 * @{
 */
#define IMU_FS_HZ 3330.0f /**< IMU 샘플링 주파수 (Hz) (3.33 kHz 근사) */
#define NFFT 1024         /**< FFT 포인트 수 (Radix-2) */
/** @} */

/**
 * @name 내부 I2C 유틸리티
 * @details LSM6DSO와 통신하기 위한 정적 래퍼(wrapper) 함수
 * @{
 */

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
 * @brief I2C로 연속 블록 읽기
 * @param reg 시작 레지스터 주소
 * @param[out] buf 데이터를 저장할 버퍼
 * @param len 읽을 바이트 수
 * @return 0 on success, 음수 에러 코드 on failure.
 */
static int rd_block(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, buf, len);
}
/** @} */

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

/**
 * @brief LSB 단위를 m/s^2 단위로 변환하기 위한 스케일 팩터를 반환합니다.
 * @details
 * ST 데이터시트 기준 (g = 9.80665 m/s^2):
 * - ±4g:  0.122 mg/LSB -> 0.122 * 9.80665 / 1000 = 0.0011964...
 * - ±16g: 0.488 mg/LSB -> 0.488 * 9.80665 / 1000 = 0.0047864...
 *
 * @param fs 현재 설정된 Full-Scale (LSM6DSO_FS_16G 또는 LSM6DSO_FS_4G)
 * @return m/s^2 per LSB (float)
 */
float lsm6dso_scale_ms2_per_lsb(lsm6dso_fs_t fs)
{
    return (fs == LSM6DSO_FS_16G) ? 0.004784f : 0.001196f; /* ±16g / ±4g 근사값 */
}

/*
 * [불필요 코드 제거]
 * 원본 117-122 라인의 중복/오해의 소지가 있는 define 삭제
 * - REG_FIFO_CTRL5 (중복)
 * - ODR_FIFO_3k33 (중복)
 * - FIFO_MODE_STOP_ON_WTM (FIFO_MODE_FIFO와 동일하여 혼란 유발)
 * (FIFO_MODE_CONTINUOUS는 상단 비트필드 정의부로 이동)
 */

/**
 * @brief LSM6DSO 센서를 초기화합니다.
 *
 * @details
 * - BDU(Block Data Update) 및 IF_INC(주소 자동 증가) 활성화
 * - I3C 인터페이스 비활성화
 * - 자이로스코프 파워 다운 (가속도계만 사용)
 * - FIFO 모드 설정 (Bypass -> Continuous)
 * - 가속도계 ODR: 3.33 kHz (0x9), FS: ±4g (fs 파라미터 무시, ±4g 고정)
 * - FIFO BDR: 3.33 kHz (0x9) (가속도계만)
 * - FIFO Watermark: @ref FIFO_WTM_WORDS (999)
 *
 * @note
 * 이 함수는 센서의 *파라미터* (ODR, FS)를 설정합니다.
 * 하지만 @ref lsm6dso_capture_once 함수는 이 함수가 설정한 FIFO 모드(Continuous) 대신,
 * 내부적으로 FIFO를 BYPASS 모드로 직접 전환하여 DRDY 폴링을 수행합니다.
 * 또한, `fs` 파라미터가 주어지지만 실제로는 `FS_XL_4G`로 고정되어 설정됩니다.
 *
 * @param fs 사용할 Full-Scale (현재 구현에서는 무시되고 ±4g로 고정됨)
 * @return 0 on success, 음수 에러 코드 on failure.
 */
int lsm6dso_init(lsm6dso_fs_t fs)
{
    (void)fs; /* 파라미터가 사용되지 않음을 명시 (FS_XL_4G로 고정됨) */

    /* 0) 소프트 리셋(선택) */
    /* wr_u8(REG_CTRL3_C, CTRL3_C_SW_RESET); k_sleep(K_MSEC(2)); */

    RC(wr_u8(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC)); // BDU=1, IF_INC=1
    RC(wr_u8(REG_CTRL9_XL, CTRL9_XL_I3C_DISABLE));        // I3C disable
    RC(wr_u8(REG_CTRL2_G, ODR_G_POWER_DOWN));             // Gyro off

    /* 1) FIFO BYPASS로 들어가서 깨끗이 비움 */
    RC(wr_u8(REG_FIFO_CTRL4, FIFO_MODE_BYPASS)); // MODE=000 (BYPASS)
    k_sleep(K_MSEC(2));

    /* 2) XL ODR/FS 설정 (3.33 kHz, ±4g) */
    /* ODR_XL = 1001b (3.33 kHz), FS = 10b (±4g) -> 0x98 */
    RC(wr_u8(REG_CTRL1_XL, (0x9 << 4) | FS_XL_4G));

    /* 3) FIFO 배치 속도(BDR) 설정 — XL만 3.333 kHz로 활성화, Gyro는 0 */
    /* BDR_GY=0000, BDR_XL=1001(3.333 kHz) -> 0x09 */
    RC(wr_u8(REG_FIFO_CTRL3, (0x0 << 4) | 0x9));

    /* 4) 워터마크(999 워드) 설정 */
    RC(wr_u8(REG_FIFO_CTRL1, (FIFO_WTM_WORDS & 0xFF)));
    RC(wr_u8(REG_FIFO_CTRL2, ((FIFO_WTM_WORDS >> 8) & 0x0F)));

    /* 5) FIFO 모드 진입 (Continuous) */
    /* @note lsm6dso_capture_once()는 이 설정을 덮어쓰고 BYPASS(0x00)를 사용함 */
    RC(wr_u8(REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS)); // MODE=110 (0x06)

    return 0;
}

/**
 * @defgroup fft_impl 내장 FFT (Fast Fourier Transform) 구현
 * @details 1024-포인트, Radix-2, 단정도 부동소수점, in-place FFT.
 * @{
 */

/* FFT 트위들 팩터(Twiddle Factor) 캐시 */
static float tw_re[NFFT / 2], tw_im[NFFT / 2];
static bool tw_ready = false; /**< 트위들 팩터 초기화 여부 */

/**
 * @brief FFT 트위들 팩터 (e^(-j*2*pi*k/N))를 미리 계산하여 캐시합니다.
 */
static void twiddle_init(void)
{
    if (tw_ready)
        return;
    for (int k = 0; k < NFFT / 2; ++k)
    {
        double ang = -2.0 * M_PI * k / (double)NFFT;
        tw_re[k] = (float)cos(ang);
        tw_im[k] = (float)sin(ang);
    }
    tw_ready = true;
}

/**
 * @brief 비트 반전(bit-reversal) 인덱스를 계산합니다.
 * @param x 원본 인덱스
 * @param log2n NFFT의 log2 (e.g., 1024 -> 10)
 * @return 비트 반전된 인덱스
 */
static unsigned bitrev(unsigned x, int log2n)
{
    unsigned n = 0;
    for (int i = 0; i < log2n; ++i)
    {
        n = (n << 1) | (x & 1);
        x >>= 1;
    }
    return n;
}

/**
 * @brief Radix-2 FFT/iFFT (in-place)
 *
 * @details
 * - 입력: `re[0..N-1]` (실수부), `im[0..N-1]` (허수부)
 * - 실신호 FFT의 경우, `im` 배열은 0으로 초기화되어야 합니다.
 * - iFFT의 경우, 정규화(1/N)가 포함됩니다.
 *
 * @param[in,out] re 실수부 배열
 * @param[in,out] im 허수부 배열
 * @param inverse 0=FFT, 0이 아니면=iFFT (역변환)
 */
static void fft_radix2(float *re, float *im, int inverse)
{
    twiddle_init();
    const int log2n = 10; /* 2^10 = 1024 */

    /* 1. Bit-reversal permutation */
    for (unsigned i = 0; i < NFFT; ++i)
    {
        unsigned j = bitrev(i, log2n);
        if (j > i)
        {
            float tr = re[i], ti = im[i];
            re[i] = re[j];
            im[i] = im[j];
            re[j] = tr;
            im[j] = ti;
        }
    }

    /* 2. Butterfly stages */
    for (unsigned len = 2; len <= NFFT; len <<= 1)
    {
        unsigned half = len >> 1;
        unsigned step = NFFT / len;
        for (unsigned i = 0; i < NFFT; i += len)
        {
            for (unsigned k = 0; k < half; ++k)
            {
                unsigned idx = k * step;
                float wr = tw_re[idx];
                float wi = inverse ? -tw_im[idx] : tw_im[idx]; /* iFFT는 공액 */
                float ur = re[i + k], ui = im[i + k];
                float vr = re[i + k + half] * wr - im[i + k + half] * wi;
                float vi = re[i + k + half] * wi + im[i + k + half] * wr;
                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + half] = ur - vr;
                im[i + k + half] = ui - vi;
            }
        }
    }

    /* 3. iFFT 정규화 */
    if (inverse)
    {
        const float invN = 1.0f / (float)NFFT;
        for (unsigned i = 0; i < NFFT; ++i)
        {
            re[i] *= invN;
            im[i] *= invN;
        }
    }
}
/** @} */ // end of fft_impl

/**
 * @defgroup signal_proc 신호 처리 유틸리티
 * @{
 */

/**
 * @brief Hann 윈도우(window)를 생성합니다.
 * @param[out] w 윈도우 값을 저장할 배열
 * @param n 윈도우 길이
 */
static void make_hann(float *w, int n)
{
    if (n <= 1)
    {
        w[0] = 1.f;
        return;
    }
    for (int i = 0; i < n; ++i)
    {
        /* w(i) = 0.5 * (1 - cos(2*pi*i / (n-1))) */
        w[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
    }
}

/**
 * @brief 주파수 대역(Hz)을 FFT bin 인덱스 범위로 변환합니다.
 * @param fs 샘플링 주파수 (Hz)
 * @param nfft FFT 포인트 수
 * @param f_lo 하한 주파수 (Hz)
 * @param f_hi 상한 주파수 (Hz)
 * @param[out] kmin 최소 bin 인덱스 (DC=0 제외, 최소 1)
 * @param[out] kmax 최대 bin 인덱스 (최대 N/2)
 */
static void band_to_bins(float fs, int nfft, float f_lo, float f_hi, int *kmin, int *kmax)
{
    int a = (int)ceilf(f_lo * nfft / fs);
    int b = (int)floorf(f_hi * nfft / fs);
    if (a < 1)
        a = 1; /* DC(0-bin) 제외 */
    if (b > nfft / 2)
        b = nfft / 2;
    if (b < a)
        b = a;
    *kmin = a;
    *kmax = b;
}

/**
 * @brief 대역 제한된 시간 파형의 RMS 및 Peak 값을 계산합니다.
 *
 * @details
 * 이 함수는 다음 단계를 수행합니다:
 * 1. LSB 데이터에서 DC(평균) 제거 및 m/s^2 스케일링
 * 2. Hann 윈도우 적용
 * 3. NFFT(1024) 포인트로 제로 패딩
 * 4. FFT 수행 (@ref fft_radix2)
 * 5. 지정된 주파수 대역(f_lo ~ f_hi) *외*의 스펙트럼을 0으로 마스킹 (대역 통과)
 * 6. iFFT 수행 (@ref fft_radix2)
 * 7. 결과로 나온 "대역 제한된 시간 파형"에서 RMS 및 Peak 값을 계산 (단위: m/s^2)
 *
 * @param lsb [in] 원시 LSB 데이터 배열
 * @param n [in] LSB 데이터 샘플 수 (최대 @ref NFFT)
 * @param lsb_to_ms2 [in] LSB-to-m/s^2 스케일 팩터
 * @param f_lo [in] 대역 하한 (Hz)
 * @param f_hi [in] 대역 상한 (Hz)
 * @param[out] out_rms_x100 [out] 계산된 RMS 값 (m/s^2 * 100)
 * @param[out] out_peak_x100 [out] 계산된 Peak 값 (m/s^2 * 100)
 */
static void bandlimited_rms_peak_ms2_x100(
    const int16_t *lsb, uint16_t n, float lsb_to_ms2,
    float f_lo, float f_hi, int16_t *out_rms_x100, int16_t *out_peak_x100)
{
    /* FFT 및 윈도우용 정적 버퍼 (스택 방지) */
    static float re[NFFT], im[NFFT], win[NFFT];
    static bool win_ready = false;
    if (!win_ready)
    {
        make_hann(win, NFFT);
        win_ready = true;
    }

    /* NFFT보다 샘플이 많으면 NFFT개만 사용 */
    const int useN = (n < NFFT) ? n : NFFT;

    /* 0) DC(평균) 제거: LSB 평균을 빼고 물리단위로 변환 */
    float mean_lsb = 0.f;
    for (int i = 0; i < useN; ++i)
        mean_lsb += lsb[i];
    mean_lsb = (useN > 0) ? (mean_lsb / useN) : 0.f;

    /* 1) 윈도우 적용 및 제로 패딩 */
    for (int i = 0; i < useN; ++i)
    {
        float v = ((float)lsb[i] - mean_lsb) * lsb_to_ms2; // 평균 제거 및 스케일링
        float w = win[i];
        re[i] = v * w; /* 창을 평균 제거 후에 곱함 */
        im[i] = 0.f;
    }
    for (int i = useN; i < NFFT; ++i)
    {
        re[i] = 0.f;
        im[i] = 0.f;
    } /* Zero-padding */

    /* 2) FFT */
    fft_radix2(re, im, 0);

    /* 3) 주파수 대역 필터링 (Band-pass) */
    int kmin = 0, kmax = 0;
    band_to_bins(IMU_FS_HZ, NFFT, f_lo, f_hi, &kmin, &kmax);

    /* 대역 외(out-of-band) 주파수 제거 (DC, Nyquist 포함) */
    for (int k = 1; k < NFFT / 2; ++k)
    {
        if (k < kmin || k > kmax)
        {
            re[k] = im[k] = 0.f;               /* Positive freq */
            re[NFFT - k] = im[NFFT - k] = 0.f; /* Negative freq (mirror) */
        }
    }
    re[0] = im[0] = 0.f;               /* DC 제거 */
    re[NFFT / 2] = im[NFFT / 2] = 0.f; /* Nyquist 제거 */

    /* 4) iFFT (시간 영역 복원) */
    fft_radix2(re, im, 1);

    /* 5) 시간영역에서 RMS/Peak 계산 (원본 샘플 길이 M=useN 기준) */
    float peak = 0.f, sumsq = 0.f;
    const int M = useN;
    for (int i = 0; i < M; ++i)
    {
        float a = re[i];
        float au = a > 0 ? a : -a; /* fabsf(a) */
        if (au > peak)
            peak = au;
        sumsq += a * a;
    }
    float rms = (M > 0) ? sqrtf(sumsq / (float)M) : 0.f;

    /* 100을 곱한 정수형으로 저장 (소수점 2자리) */
    *out_peak_x100 = (int16_t)(peak * 100.0f + 0.5f);
    *out_rms_x100 = (int16_t)(rms * 100.0f + 0.5f);
}
/** @} */ // end of signal_proc

/**
 * @brief 3.33kHz ODR에서 DRDY 폴링을 사용하여 가속도 데이터를 캡처하고 통계를 계산합니다.
 *
 * @details
 * 이 함수는 @ref lsm6dso_init 에서 설정한 FIFO 모드를 무시하고,
 * FIFO를 BYPASS 모드로 직접 설정하여 DRDY(Data Ready) 폴링 방식을 사용합니다.
 *
 * 1. FIFO를 BYPASS 모드로 설정합니다.
 * 2. 약 330ms 동안 또는 최대 1024개 샘플을 수집할 때까지
 * @ref REG_STATUS_REG 의 XLDA 비트를 폴링(polling)합니다.
 * 3. 새 데이터가 준비되면 @ref REG_OUTX_L_A 부터 6바이트를 읽어
 * @ref g_ax, @ref g_ay, @ref g_az 전역 버퍼에 저장합니다.
 * 4. 캡처가 완료되면 전체 대역(Broadband)의 RMS/Peak (m/s^2)를 계산합니다.
 * 5. @ref bandlimited_rms_peak_ms2_x100 를 호출하여
 * 10-1000 Hz 대역의 RMS/Peak (m/s^2)를 계산합니다.
 * 6. 모든 결과를 @ref lsm6dso_stats_t 구조체에 채웁니다.
 *
 * @param[out] out 통계 결과를 저장할 @ref lsm6dso_stats_t 구조체 포인터
 * @return 0 on success (최소 1개 샘플 수집), -EINVAL if out is NULL,
 * -EIO if no samples collected, or I2C 에러 코드.
 */
int lsm6dso_capture_once(lsm6dso_stats_t *out)
{
    if (!out)
        return -EINVAL;
    memset(out, 0, sizeof(*out));

    int rc;

    /* A) FIFO 완전 비활성 (BYPASS) — DRDY 폴링을 위함 */
    rc = wr_u8(REG_FIFO_CTRL4, FIFO_MODE_BYPASS); /* 0x00 */
    if (rc)
        return rc;
    k_sleep(K_MSEC(2));

    /* B) 0.3 s 동안 DRDY 폴링하며 직접 읽기
          - ODR=3.33 kHz 이므로 최대 ≈ 1100 샘플 가능
          - 안전상 최대 1024개까지만 저장 */
    const uint32_t window_ms = 330;
    const uint16_t N_MAX = 1024;

    uint32_t t0 = k_uptime_get_32();
    uint16_t n_accel = 0;

    while ((k_uptime_get_32() - t0) < window_ms)
    {
        uint8_t sr = 0;
        rc = rd_u8(REG_STATUS_REG, &sr);
        if (rc)
            return rc;

        if (sr & XLDA_BIT)
        {
            /* 새 데이터 있음: OUTX_L_A .. OUTZ_H_A (연속 6바이트) 읽기 */
            uint8_t reg = REG_OUTX_L_A;
            uint8_t raw[6];
            rc = i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, raw, sizeof(raw));
            if (rc)
                return rc;

            int16_t x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
            int16_t y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
            int16_t z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

            /* 전역 버퍼에 저장 (버퍼 크기 한도 내) */
            if (n_accel < FIFO_WTM_WORDS && n_accel < N_MAX)
            {
                g_ax[n_accel] = x;
                g_ay[n_accel] = y;
                g_az[n_accel] = z;
                n_accel++;
            }
        }
        else
        {
            /* 데이터 없음: ODR 주기(약 300us)보다 짧게 대기 */
            k_busy_wait(80); /* 80 us (ODR 주기의 약 1/4) */
        }

        /* 999개 또는 1024개 모이면 중지 */
        if (n_accel >= FIFO_WTM_WORDS || n_accel >= N_MAX)
            break;
    }

    out->n = n_accel;
    out->wtm_reached = (n_accel >= FIFO_WTM_WORDS);

    /* C) 통계 계산 (전체대역, Broadband) */
    /* init에서 FS_XL_4G로 고정했으므로 해당 스케일 사용 */
    float s = lsm6dso_scale_ms2_per_lsb(LSM6DSO_FS_4G);
    float sumsq_x = 0, sumsq_y = 0, sumsq_z = 0, peak_x = 0, peak_y = 0, peak_z = 0;

    for (uint16_t i = 0; i < n_accel; ++i)
    {
        float fx = g_ax[i] * s, fy = g_ay[i] * s, fz = g_az[i] * s;
        float axu = fabsf(fx), ayu = fabsf(fy), azu = fabsf(fz);
        if (axu > peak_x)
            peak_x = axu;
        if (ayu > peak_y)
            peak_y = ayu;
        if (azu > peak_z)
            peak_z = azu;
        sumsq_x += fx * fx;
        sumsq_y += fy * fy;
        sumsq_z += fz * fz;
    }
    float rms_x = (n_accel > 0) ? sqrtf(sumsq_x / n_accel) : 0.f;
    float rms_y = (n_accel > 0) ? sqrtf(sumsq_y / n_accel) : 0.f;
    float rms_z = (n_accel > 0) ? sqrtf(sumsq_z / n_accel) : 0.f;

    /* m/s^2 * 100 값으로 저장 */
    out->peak_ms2_x100[0] = (int16_t)(peak_x * 100.f + 0.5f);
    out->peak_ms2_x100[1] = (int16_t)(peak_y * 100.f + 0.5f);
    out->peak_ms2_x100[2] = (int16_t)(peak_z * 100.f + 0.5f);
    out->rms_ms2_x100[0] = (int16_t)(rms_x * 100.f + 0.5f);
    out->rms_ms2_x100[1] = (int16_t)(rms_y * 100.f + 0.5f);
    out->rms_ms2_x100[2] = (int16_t)(rms_z * 100.f + 0.5f);

    /* D) 10–1000 Hz 대역 제한 통계 계산 */
    const float FLO = 10.0f, FHI = 1000.0f;
    bandlimited_rms_peak_ms2_x100(g_ax, n_accel, s, FLO, FHI,
                                  &out->bl_rms_ms2_x100[0], &out->bl_peak_ms2_x100[0]);
    bandlimited_rms_peak_ms2_x100(g_ay, n_accel, s, FLO, FHI,
                                  &out->bl_rms_ms2_x100[1], &out->bl_peak_ms2_x100[1]);
    bandlimited_rms_peak_ms2_x100(g_az, n_accel, s, FLO, FHI,
                                  &out->bl_rms_ms2_x100[2], &out->bl_peak_ms2_x100[2]);

    rd_u8(REG_WHO_AM_I, &out->whoami);

    return (out->n > 0) ? 0 : -EIO; /* 샘플이 하나도 없으면 에러 */
}