/**
 * @file lsm6dso.c
 * @brief LSM6DSO IMU 드라이버
 *
 * @details
 * LSM6DSO 가속도계 센서 드라이버
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

#define RC_CHECK(rc) ({ \
    int __rc = (rc); \
    if (__rc != 0) { \
        LOG_ERR("I2C Error: %d at %s:%d", __rc, __func__, __LINE__); \
        __rc = 1; /* 이 매크로 자체의 반환값 (성공 0, 실패 1) */ \
    } else { \
        __rc = 0; \
    } \
    __rc; \
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
 * @brief 가속도 ODR: ODR_FIFO code for 3.33 kHz is 0x0A.
 * @note @ref lsm6dso_init 에서는 0x9 (3.33kHz)를 사용합니다. 이 값은 [7:3] 비트에 배치되므로 반드시 <<3 해줘야 함!
 */
#define ODR_FIFO_3k33_SH ((uint8_t)(0x09 << 3))
#define FS_XL_4G (0x2 << 2)  /**< 가속도 Full-Scale: ±4g (10b) */
#define FS_XL_16G (0x1 << 2) /**< 가속도 Full-Scale: ±16g (01b) */

/* REG_CTRL2_G: Gyroscope */
#define ODR_G_POWER_DOWN (0x0 << 4) /**< 자이로스코프 파워 다운 */

/* FIFO 관련 설정 */
// #define ODR_FIFO_3k33 0x0A /**< FIFO ODR: 3.33 kHz (1010b) */
/**
 * @brief FIFO 가속도 BDR: 3.33 kHz (1010b)
 * @note @ref lsm6dso_init 에서는 0x9 (3.33kHz)를 사용합니다.
 */
#define BDR_XL_3k33 (0x0A)
#define FIFO_CTRL4_STOP_ON_WTM BIT(5) /**< Watermark 도달 시 FIFO 중지 (0=Overwrite) */
#define FIFO_MODE_BYPASS 0x0          /**< FIFO 모드: Bypass (000b) */
#define FIFO_MODE_FIFO 0x1            /**< FIFO 모드: FIFO (001b) */
#define FIFO_MODE_CONTINUOUS 0x6      /**< FIFO 모드: Continuous (110b) */
#define STOP_ON_WTM_BIT BIT(5)
#define ACC_TAG 0x01 /* XL Tag 값 (하위 nibble) */

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

int rd_u16(uint8_t reg, uint16_t *data_out)
{
    uint8_t raw_data[2]; // LSB, MSB 순서로 데이터를 저장할 배열
    int rc;

    // 1. 레지스터 주소(1바이트)를 쓰고, 2바이트를 읽습니다.
    rc = i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, raw_data, 2);
    if (rc)
        return rc;

    // 2. Little-Endian 순서로 16비트를 재구성합니다.
    // LSB (raw_data[0]) + (MSB (raw_data[1]) << 8)
    *data_out = (uint16_t)raw_data[0] | ((uint16_t)raw_data[1] << 8);

    return 0; // 성공
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

static inline int fifo_set_mode(uint8_t mode)
{
    /* 항상 ODR=3.33kHz(0x09<<3=0x48) + mode */
    return wr_u8(REG_FIFO_CTRL5, (uint8_t)(ODR_FIFO_3k33_SH | (mode & 0x07)));
}

static inline int fifo_expect_ctrl5(const struct shell *sh, uint8_t expect)
{
    uint8_t v = 0;
    int rc = rd_u8(REG_FIFO_CTRL5, &v);
    if (rc)
    {
        if (sh)
            shell_print(sh, "rd CTRL5 rc=%d", rc);
        return rc;
    }
    if (v != expect)
    {
        if (sh)
            shell_print(sh, "CTRL5 mismatch: got 0x%02X, want 0x%02X", v, expect);
        return -EIO;
    }
    return 0;
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
 * 하지만 @ref lsm6dso_capture_once 함수는 이 함수가 설정한 FIFO 모드(Continuous) 대신, 내부적으로 FIFO를 BYPASS 모드로 직접 전환하여 DRDY 폴링을 수행합니다. 또한, `fs` 파라미터가 주어지지만 실제로는 `FS_XL_4G`로 고정되어 설정됩니다.
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

    /* 타임스탬프 끄기 + 가속도만 배치 */
    RC(wr_u8(REG_CTRL10_C, 0x00));   /* TIMESTAMP_EN=0 */
    RC(wr_u8(REG_FIFO_CTRL3, 0x09)); /* XL only @ 3.33kHz, Gyro off */

    /* 2) XL ODR/FS 설정 (3.33 kHz, ±4g) */
    /* 3.33 kHz, ±4g  (ODR_XL=0b1010, FS=±4g → 0xA8) */
    RC(wr_u8(REG_CTRL1_XL, 0x98));

    /* 안전하게 BYPASS로 두고 시작 (ODR_FIFO=3.33kHz 설정은 유지) */
    RC(wr_u8(REG_FIFO_CTRL5, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_BYPASS)));

    /* 3) FIFO 배치 속도(BDR) 설정 — XL만 3.333 kHz로 활성화, Gyro는 0 */
    /* BDR_GY=0000, BDR_XL=1001(3.333 kHz) -> 0x09 */
    // RC(wr_u8(REG_FIFO_CTRL3, (0x0 << 4) | 0x9));

    /* 4) 워터마크(999 워드) 설정 */
    RC(wr_u8(REG_FIFO_CTRL1, (FIFO_WTM_WORDS & 0xFF)));
    RC(wr_u8(REG_FIFO_CTRL2, ((FIFO_WTM_WORDS >> 8) & 0x0F)));

    /* 5) FIFO 모드 진입 (Continuous) */
    /* @note lsm6dso_capture_once()는 이 설정을 덮어쓰고 BYPASS(0x00)를 사용함 */
    RC(wr_u8(REG_FIFO_CTRL5, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_CONTINUOUS)));

    /* 레지스터 변경이 안되서 재 입력 */
    RC(wr_u8(REG_CTRL10_C, 0x00));           /* 타임스탬프 OFF */
    RC(wr_u8(REG_FIFO_CTRL3, 0x09));         /* 가속도만 라우팅 */
    RC(fifo_set_mode(FIFO_MODE_CONTINUOUS)); /* => CTRL5 = 0x48|0x06 = 0x4E */

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

/* 가속도 TAG 값(하위 nibble) — 대부분 0x01, 일부 리비전/설정에서 0x02일 수도 있어
 * 우선 0x01을 기본으로 하되, 청크에서 다수결로 동적으로 검출하여 사용
 */
// static uint8_t detect_acc_tag(const uint8_t *buf, size_t len)
// {
//     int cnt1=0, cnt2=0;
//     for (size_t off=0; off+7 <= len && off < 28; ++off) {
//         uint8_t t = buf[off] & 0x0F;
//         if (t == 0x01) cnt1++;
//         if (t == 0x02) cnt2++;
//     }
//     return (cnt2 > cnt1) ? 0x02 : 0x01;
// }

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/* TAG 후보 검출 (가속도는 보통 0x01, 환경에 따라 0x02 케이스도 있어 다수결로 선택) */
static uint8_t detect_acc_tag(const uint8_t *buf, size_t len)
{
    int c1 = 0, c2 = 0;
    size_t probe = MIN(len, (size_t)64);
    for (size_t i = 0; i + 7 <= probe; ++i)
    {
        uint8_t t = buf[i] & 0x0F;
        if (t == 0x01)
            c1++;
        else if (t == 0x02)
            c2++;
    }
    return (c2 > c1) ? 0x02 : 0x01;
}

/* 청크 내에서 'acc_tag'가 7바이트 간격으로 반복되는 시작 오프셋을 찾는다.
 * 못 찾으면 0 리턴(가장 앞에서부터 시도).
 */
// static size_t find_sync_7B(const uint8_t *buf, size_t len, uint8_t acc_tag)
// {
//     /* 후보 오프셋 0..6 을 시험 */
//     for (size_t base=0; base<7 && base+7 <= len; ++base) {
//         size_t ok=0;
//         for (size_t i=base; i+7 <= len; i+=7) {
//             if ( (buf[i] & 0x0F) == acc_tag ) { ok++; }
//             else break;
//         }
//         if (ok >= 2) return base; /* 최소 2패킷 이상 연속이면 신뢰 */
//     }
//     return 0;
// }

/* 7바이트 간격으로 TAG가 반복되는 시작 오프셋 찾기 (간단 휴리스틱) */
static size_t find_sync_7B(const uint8_t *buf, size_t len, uint8_t acc_tag)
{
    size_t limit = MIN(len, (size_t)56);
    for (size_t base = 0; base < 7 && base + 14 <= limit; ++base)
    {
        size_t ok = 0;
        for (size_t i = base; i + 7 <= len; i += 7)
        {
            if ((buf[i] & 0x0F) == acc_tag)
                ok++;
            else
                break;
        }
        if (ok >= 2)
            return base;
    }
    return 0;
}

/* 7B 패턴이 잘 맞는 시작 오프셋 찾기 (TAG 값은 무시) */
static size_t find_sync_7B_loose(const uint8_t *buf, size_t len)
{
    for (size_t base = 0; base < 7 && base + 14 <= len; ++base)
    {
        size_t ok = 0;
        for (size_t i = base; i + 7 <= len; i += 7)
        {
            /* X/Y/Z가 전부 0x0000 이면 노이즈일 수 있으니 2~3세트는 통과시키고,
               이후 6바이트(LSB/HB) 값이 '변화'하는지로 판단 */
            if (i + 7 > len)
                break;
            ok++;
        }
        if (ok >= 2)
            return base;
    }
    return 0;
}

/**
 * @brief 3.33kHz ODR에서 DRDY 폴링을 사용하여 가속도 데이터를 캡처하고 통계를 계산합니다.
 *
 * @details
 * 이 함수는 FIFO를 BYPASS 모드로 직접 설정하여 DRDY(Data Ready) 폴링 방식을 사용합니다.
 *
 * 1. FIFO를 BYPASS 모드로 설정합니다.
 * 2. 약 330ms 동안 또는 최대 999개 샘플을 수집할 때까지
 * REG_STATUS_REG (0x1E)의 XLDA 비트(BIT 0)를 폴링(polling)합니다.
 * 3. 새 데이터(XLDA=1)가 준비되면 REG_OUTX_L_A (0x28)부터 6바이트를 읽어
 * g_ax, g_ay, g_az 전역 버퍼에 저장합니다.
 * 4. 캡처가 완료되면 전체 대역(Broadband)의 RMS/Peak (m/s^2)를 계산합니다.
 * 5. bandlimited_rms_peak_ms2_x100 를 호출하여
 * 10-1000 Hz 대역의 RMS/Peak (m/s^2)를 계산합니다.
 * 6. 모든 결과를 lsm6dso_stats_t 구조체에 채웁니다.
 *
 * @param[out] out 통계 결과를 저장할 lsm6dso_stats_t 구조체 포인터
 * @return 0 on success (최소 1개 샘플 수집), -EINVAL if out is NULL,
 * -EIO if no samples collected, or I2C 에러 코드.
 */
int lsm6dso_capture_once(lsm6dso_stats_t *out)
{
    if (!out)
        return -EINVAL;
    memset(out, 0, sizeof(*out));

    /* 0) FIFO BYPASS 모드로 설정 (ODR_FIFO는 3.33k 유지) */
    RC(fifo_set_mode(FIFO_MODE_BYPASS));
    k_sleep(K_MSEC(2));
    /* BYPASS 모드(0x00) + ODR 3.33k(0x48) = 0x48 확인 */
    RC(fifo_expect_ctrl5(NULL, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_BYPASS)));

    const uint16_t TARGET_N = 999;
    uint16_t n = 0;
    uint8_t raw[6]; /* X,Y,Z (L/H) */
    uint8_t reg_status = REG_STATUS_REG; /* 0x1E */
    uint8_t reg_data = REG_OUTX_L_A;     /* 0x28 */
    uint8_t status_val = 0;

    /* * 1) DRDY 폴링 루프 
     * 3.33kHz에서 999개 샘플은 약 300ms 소요.
     * 샘플당 2ms, 전체 600ms의 타임아웃 설정 (I2C 오버헤드 감안).
     */
    uint32_t t_start = k_uptime_get_32();
    const uint32_t CAPTURE_TIMEOUT_MS = 600;
    const int POLL_TIMEOUT_US = 2000; /* 샘플당 2ms 타임아웃 */

    while (n < TARGET_N)
    {
        /* 전체 캡처 타임아웃 확인 */
        if (k_uptime_get_32() - t_start > CAPTURE_TIMEOUT_MS)
        {
            LOG_WRN("DRDY poll timeout: got %u/%u samples", n, TARGET_N);
            break; 
        }

        /* XLDA (Bit 0) 대기 */
        int poll_retries = POLL_TIMEOUT_US / 10; /* 10us 간격으로 폴링 */
        status_val = 0;
        while (poll_retries-- > 0)
        {
            RC(rd_u8(reg_status, &status_val));
            if (status_val & XLDA_BIT)
                break;
            k_busy_wait(10); /* 10us 대기 */
        }

        if (!(status_val & XLDA_BIT))
        {
            /* 샘플 1개에 대한 타임아웃 발생 */
            LOG_WRN("XLDA poll timeout (sample %u)", n);
            break;
        }

        /* 2) 데이터 6바이트 읽기 (OUTX_L_A 부터 연속) */
        RC(rd_block(reg_data, raw, 6));

        /* 3) 전역 버퍼에 저장 */
        g_ax[n] = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
        g_ay[n] = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
        g_az[n] = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
        n++;
    }

    out->n = n;
    out->wtm_reached = 0; /* WTM 모드를 사용하지 않았으므로 0 */

    /* 4) DC 제거 후 전체대역 통계 (기존 코드와 동일) */
    float s = lsm6dso_scale_ms2_per_lsb(LSM6DSO_FS_4G);
    float mx = 0, my = 0, mz = 0;
    for (uint16_t i = 0; i < n; ++i)
    {
        mx += g_ax[i];
        my += g_ay[i];
        mz += g_az[i];
    }
    if (n > 0)
    {
        mx /= n;
        my /= n;
        mz /= n;
    }
    float sx = 0, sy = 0, sz = 0, px = 0, py = 0, pz = 0;
    for (uint16_t i = 0; i < n; ++i)
    {
        float x = ((float)g_ax[i] - mx) * s, y = ((float)g_ay[i] - my) * s, z = ((float)g_az[i] - mz) * s;
        float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);
        if (ax > px)
            px = ax;
        if (ay > py)
            py = ay;
        if (az > pz)
            pz = az;
        sx += x * x;
        sy += y * y;
        sz += z * z;
    }
    float rx = (n > 0) ? sqrtf(sx / n) : 0.f, ry = (n > 0) ? sqrtf(sy / n) : 0.f, rz = (n > 0) ? sqrtf(sz / n) : 0.f;
    out->peak_ms2_x100[0] = (int16_t)(px * 100.f + 0.5f);
    out->peak_ms2_x100[1] = (int16_t)(py * 100.f + 0.5f);
    out->peak_ms2_x100[2] = (int16_t)(pz * 100.f + 0.5f);
    out->rms_ms2_x100[0] = (int16_t)(rx * 100.f + 0.5f);
    out->rms_ms2_x100[1] = (int16_t)(ry * 100.f + 0.5f);
    out->rms_ms2_x100[2] = (int16_t)(rz * 100.f + 0.5f);

    /* 5) 10–1000 Hz 대역 제한 (기존 코드와 동일) */
    const float FLO = 10.f, FHI = 1000.f;
    bandlimited_rms_peak_ms2_x100(g_ax, n, s, FLO, FHI, &out->bl_rms_ms2_x100[0], &out->bl_peak_ms2_x100[0]);
    bandlimited_rms_peak_ms2_x100(g_ay, n, s, FLO, FHI, &out->bl_rms_ms2_x100[1], &out->bl_peak_ms2_x100[1]);
    bandlimited_rms_peak_ms2_x100(g_az, n, s, FLO, FHI, &out->bl_rms_ms2_x100[2], &out->bl_peak_ms2_x100[2]);

    RC(rd_u8(REG_WHO_AM_I, &out->whoami));

    /* 6) 모드 복구 (lsm6dso_init의 기본 상태인 Continuous로 복구) */
    RC(fifo_set_mode(FIFO_MODE_CONTINUOUS));
    RC(fifo_expect_ctrl5(NULL, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_CONTINUOUS)));
    
    return (out->n > 0) ? 0 : -EIO;
}

/* ===== FIFO dump (diagnostic) ===== */

static void dump_hex_lines(const struct shell *sh, const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i += 16)
    {
        char line[16 * 3 + 8];
        size_t k = 0;
        k += snprintk(line + k, sizeof(line) - k, "%04x: ", (uint32_t)i);
        size_t m = MIN((size_t)16, n - i);
        for (size_t j = 0; j < m; ++j)
            k += snprintk(line + k, sizeof(line) - k, "%02X ", p[i + j]);
        shell_print(sh, "%s", line);
    }
}

/* FIFO 덤프: STOP_ON_WTM|FIFO로 채운 뒤 bytes_req 바이트 버스트 읽기 → RAW+파싱 출력 */
int lsm6dso_dump_fifo(const struct shell *shell, uint16_t bytes_req)
{
    if (bytes_req == 0)
        bytes_req = 224;           /* 디폴트 224B */
    bytes_req -= (bytes_req % 7u); /* 7의 배수로 정렬 */
    if (bytes_req < 7)
        bytes_req = 7;
    if (bytes_req > 420)
        bytes_req = 420; /* 과도한 로그 방지 (최대 60패킷) */

    /* 0) 완전 초기화: BYPASS + ODR 설정 고정 */
    RC(fifo_set_mode(FIFO_MODE_BYPASS));
    k_sleep(K_MSEC(2));
    RC(wr_u8(REG_CTRL10_C, 0x00));                                                /* TIMESTAMP_EN=0 */
    RC(wr_u8(REG_FIFO_CTRL3, 0x09));                                              /* XL only @ 3.33kHz */
    RC(fifo_expect_ctrl5(shell, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_BYPASS))); /* 기대: 0x48 */

    /* 1) 워터마크 설정 (word 단위) */
    uint16_t wtm_words = (bytes_req / 2) + 64; /* 224B -> 112 + 여유 */
    RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(wtm_words & 0xFF)));
    RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((wtm_words >> 8) & 0x0F)));

    /* 2) STOP_ON_WTM 설정(CTRL4) */
    RC(wr_u8(REG_FIFO_CTRL4, STOP_ON_WTM_BIT));

    /* 3) FIFO 시작 (여기가 진짜 핵심) */
    RC(fifo_set_mode(FIFO_MODE_FIFO));
    RC(fifo_expect_ctrl5(shell, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_FIFO))); /* 기대: 0x49 */

    /* 3.5) 시작 직후 상태 한 번 더 읽어보기(진단) */
    uint8_t st12[2] = {0};
    RC(i2c_burst_read(i2c0, LSM6DSO_I2C_ADDR, REG_FIFO_STATUS1, st12, 2));
    uint16_t diff_w = ((uint16_t)(st12[1] & 0x0F) << 8) | st12[0];
    if (diff_w == 0)
    {
        uint8_t c5 = 0, c4 = 0;
        rd_u8(REG_FIFO_CTRL5, &c5);
        rd_u8(REG_FIFO_CTRL4, &c4);
        shell_print(shell, "after start: CTRL5=0x%02X, CTRL4=0x%02X, DIFF=%u", c5, c4, diff_w);
    }

    /* 4) DIFF 상승 대기 */
    for (int tries = 0; tries < 600; ++tries)
    { /* 최대 ~600ms */
        uint8_t st[2];
        RC(i2c_burst_read(i2c0, LSM6DSO_I2C_ADDR, REG_FIFO_STATUS1, st, 2));
        diff_w = ((uint16_t)(st[1] & 0x0F) << 8) | st[0];
        if (diff_w)
            break; /* 최소 1 word라도 쌓이면 진행 */
        k_sleep(K_MSEC(2));
    }
    uint32_t bytes_avail = (uint32_t)diff_w * 2u;
    /* 최소 7B는 읽되, 7의 배수 유지 */
    uint16_t bytes_now = (uint16_t)MIN((uint32_t)bytes_req, bytes_avail);
    if (bytes_now < 7u)
        bytes_now = 7u;
    bytes_now -= (bytes_now % 7u);

    static uint8_t buf[420];
    uint8_t reg = REG_FIFO_DATA_OUT_TAG;
    int rc = i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, buf, bytes_now);
    if (rc)
    {
        shell_print(shell, "dump: i2c rc=%d", rc);
        return rc;
    }

    /* 4) TAG 검출 + 오프셋 동기화 */
    uint8_t acc_tag = detect_acc_tag(buf, bytes_now);
    size_t off = find_sync_7B(buf, bytes_now, acc_tag);

    /* 5) RAW HEX 출력 */
    shell_print(shell, "FIFO dump: req=%uB, got=%uB, DIFF≈%uB, ACC_TAG=0x%02X, sync_off=%u",
                (unsigned)bytes_req, (unsigned)bytes_now, (unsigned)bytes_avail,
                acc_tag, (unsigned)off);
    dump_hex_lines(shell, buf, bytes_now);

    /* 6) 패킷 파싱(최대 32 패킷) */
    uint16_t shown = 0;
    for (size_t i = off; i + 7 <= bytes_now && shown < 32; i += 7)
    {
        int16_t x = (int16_t)((uint16_t)buf[i + 1] | ((uint16_t)buf[i + 2] << 8));
        int16_t y = (int16_t)((uint16_t)buf[i + 3] | ((uint16_t)buf[i + 4] << 8));
        int16_t z = (int16_t)((uint16_t)buf[i + 5] | ((uint16_t)buf[i + 6] << 8));
        shell_print(shell, "[%02u] X=%6d  Y=%6d  Z=%6d  (LSB)", shown, x, y, z);
        shown++;
    }
    shell_print(shell, "parsed=%u pkt (of %uB)", shown, bytes_now);

    /* 7) 모드 복구(원하면 Continuous) */
    /* 복구: Continuous @ 3.33kHz */
    RC(fifo_set_mode(FIFO_MODE_CONTINUOUS));
    RC(fifo_expect_ctrl5(shell, (uint8_t)(ODR_FIFO_3k33_SH | FIFO_MODE_CONTINUOUS))); /* 기대: 0x4E */

    return 0;
}
