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
#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

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
#define RC(expr)       \
  do                   \
  {                    \
    int __rc = (expr); \
    if (__rc)          \
      return __rc;     \
  } while (0)

// I2C 반환 코드를 로그로 출력하는 매크로 (오류 발생 시 디버그 정보 추가)
#define LOG_RC(expr, name)                                                    \
  ({                                                                          \
    int __rc = (expr);                                                        \
    if (__rc != 0)                                                            \
    {                                                                         \
      LOG_ERR("I2C Error [%s]: %d at %s:%d", name, __rc, __func__, __LINE__); \
    }                                                                         \
    __rc;                                                                     \
  })

#define RC_CHECK(rc)                                               \
  ({                                                               \
    int __rc = (rc);                                               \
    if (__rc != 0)                                                 \
    {                                                              \
      LOG_ERR("I2C Error: %d at %s:%d", __rc, __func__, __LINE__); \
      __rc = 1; /* 이 매크로 자체의 반환값 (성공 0, 실패 1) */     \
    }                                                              \
    else                                                           \
    {                                                              \
      __rc = 0;                                                    \
    }                                                              \
    __rc;                                                          \
  })

/**
 * @name 전역 캡처 버퍼
 * @details
 * 쉘 스레드 등의 스택 오버플로우를 방지하기 위해 BSS 섹션에 큰 버퍼를 전역으로 할당합니다. 이 버퍼들은 @ref lsm6dso_capture_once 및 FFT 처리(@ref bandlimited_rms_peak_ms2_x100)에서 사용됩니다.
 * @note 이름과 달리 실제로는 FIFO가 아닌 DRDY 폴링 캡처에 사용됩니다.
 * @{
 */
#define FIFO_WTM_WORDS 500    /**< 캡처할 최대 샘플 수 (워드) - 9bit(512) 제한 근사 */
#define FIFO_SLACK_WORDS 12   /**< TAG 불일치 대비 여유 읽기량 (워드) */
#define FIFO_BYTES_PER_WORD 7 /**< FIFO에서 가속도 1샘플당 7B (XYZ+TAG) */
#define FIFO_TAG_OFFSET 0     /**< FIFO 워드 내 TAG 위치 (첫 바이트) */

/* 캡처 대상 배열 (BSS) */
#ifndef FIFO_CAPTURE_REPEAT
#define FIFO_CAPTURE_REPEAT 2 /**< FIFO 캡처 반복 횟수 (총 1000 샘플 목표) */
#define FIFO_TOTAL_WORDS (FIFO_WTM_WORDS * FIFO_CAPTURE_REPEAT)
#endif
static int16_t g_ax[FIFO_TOTAL_WORDS]; /**< X축 가속도 LSB 데이터 버퍼 */
static int16_t g_ay[FIFO_TOTAL_WORDS]; /**< Y축 가속도 LSB 데이터 버퍼 */
static int16_t g_az[FIFO_TOTAL_WORDS]; /**< Z축 가속도 LSB 데이터 버퍼 */

/* FIFO RAW 버퍼: XYZ+TAG 순서 그대로 받아두는 용도
 * 목표 샘플(FIFO_WTM_WORDS)보다 약간 큰 여유(FIFO_SLACK_WORDS)를 두어 다른 TAG 패킷이 섞여도 목표 갯수를 확보 */
static uint8_t g_fifo_raw[(FIFO_WTM_WORDS + FIFO_SLACK_WORDS) * FIFO_BYTES_PER_WORD];
static float g_cal_offset_lsb[3] = {0.f, 0.f, 0.f}; /**< 축별 보정 오프셋(LSB) */
static bool g_calc_acc = true;
static bool g_calc_vel = true;

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
 * @note @ref lsm6dso_init 에서는 0x9 (3.33kHz)를 사용합니다. 이 값은 [7:3]
 * 비트에 배치되므로 반드시 <<3 해줘야 함!
 */
#define ODR_FIFO_3k33_SH ((uint8_t)(0x09 << 3))
#define FS_XL_4G (0x2 << 2)  /**< 가속도 Full-Scale: ±4g (10b) */
#define FS_XL_16G (0x1 << 2) /**< 가속도 Full-Scale: ±16g (01b) */
#define CTRL1_ODR_3k33 0xA0  /**< CTRL1_XL ODR bits for 3.33 kHz */
#define SENS_LSB_TO_MS2_4G 0.001196f
#define SENS_LSB_TO_MS2_16G 0.004784f
#define PSD_N 999
#define PSD_K_MIN 3
#define PSD_K_MAX 300
#define PSD_HANN_CG 0.4994994995f
#define PSD_HANN_U 0.3746246246f
#define G_CONST_MS2 9.80665f
#define MMPS_X100(v_mps) ((int16_t)((v_mps * 100000.0f) + ((v_mps >= 0.f) ? 0.5f : -0.5f)))
#define MS2_X100(v_ms2) ((int16_t)((v_ms2 * 100.0f) + ((v_ms2 >= 0.f) ? 0.5f : -0.5f)))
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

/* 공통 설정 래퍼 */
static int lsm6dso_set_fs(lsm6dso_scale_t scale)
{
  const bool use_16g = (scale == LSM6DSO_SCALE_16G);
  const uint8_t fs_bits = use_16g ? FS_XL_16G : FS_XL_4G;
  return wr_u8(REG_CTRL1_XL, (uint8_t)(CTRL1_ODR_3k33 | fs_bits));
}

static int lsm6dso_apply_fifo_base(void)
{
  /* 타임스탬프 OFF, XL만 FIFO에 배치, STOP_ON_WTM=0, 워터마크 설정 */
  RC(wr_u8(REG_CTRL10_C, 0x00));
  RC(wr_u8(REG_FIFO_CTRL3, 0x09)); /* XL only @ 3.33kHz, Gyro off */
  RC(wr_u8(REG_FIFO_CTRL4, STOP_ON_WTM_BIT)); /* WTM 도달 시 정지 */
  RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(FIFO_WTM_WORDS & 0xFF)));
  RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((FIFO_WTM_WORDS >> 8) & 0x0F)));
  return 0;
}

static inline int fifo_set_mode(uint8_t mode)
{
  int rc;
  uint8_t ctrl4 = 0;

  /* 1. ODR 설정: FIFO_CTRL5 (0x0B) bits[6:3] 기존 코드에서 mode가 CTRL5에 들어가는 문제 수정 */
  rc = wr_u8(REG_FIFO_CTRL5, ODR_FIFO_3k33_SH);
  if (rc)
    return rc;

  /* 2. Mode 설정: FIFO_CTRL4 (0x0A) bits[2:0] 기존 설정을 유지하면서 Mode 비트만 변경 (RMW) */
  rc = rd_u8(REG_FIFO_CTRL4, &ctrl4);
  if (rc)
    return rc;

  ctrl4 &= ~0x07;         /* 하위 3비트(Mode) 클리어 */
  ctrl4 |= (mode & 0x07); /* 새 Mode 설정 */

  return wr_u8(REG_FIFO_CTRL4, ctrl4);
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
 * @brief LSM6DSO 센서를 초기화합니다.
 *
 * @details
 * - BDU(Block Data Update) 및 IF_INC(주소 자동 증가) 활성화
 * - I3C 인터페이스 비활성화
 * - 자이로스코프 파워 다운 (가속도계만 사용)
 * - FIFO 모드 설정 (Bypass -> Continuous)
 * - 가속도계 ODR: 3.33 kHz (0x9), 기본 FS: ±4g
 * - FIFO BDR: 3.33 kHz (0x9) (가속도계만)
 * - FIFO Watermark: @ref FIFO_WTM_WORDS (500)
 *
 * @note
 * 이 함수는 센서의 *파라미터* (ODR, FS)를 설정합니다.
 * 하지만 @ref lsm6dso_capture_once 함수는 캡처 시 FIFO 모드를 BYPASS로 전환하고, 호출 시 scale 파라미터(±4g/±16g)에 맞춰 FS를 다시 설정합니다.
 *
 * @return 0 on success, 음수 에러 코드 on failure.
 */
int lsm6dso_init()
{
  /* 0) 소프트 리셋(선택) */
  /* wr_u8(REG_CTRL3_C, CTRL3_C_SW_RESET); k_sleep(K_MSEC(2)); */

  RC(wr_u8(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC)); // BDU=1, IF_INC=1
  RC(wr_u8(REG_CTRL9_XL, CTRL9_XL_I3C_DISABLE));        // I3C disable
  RC(wr_u8(REG_CTRL2_G, ODR_G_POWER_DOWN));             // Gyro off

  /* 기본 FS=±4g, 3.33kHz */
  RC(lsm6dso_set_fs(LSM6DSO_SCALE_4G));

  /* FIFO 기본 설정 (WTM, STOP_ON_WTM=0, XL만 배치) */
  RC(lsm6dso_apply_fifo_base());

  /* BYPASS로 초기화 후 안정 대기 */
  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  k_sleep(K_MSEC(5));

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
 * 7. 결과로 나온 "대역 제한된 시간 파형"에서 RMS 및 Peak 값을 계산 (단위:
 * m/s^2)
 *
 * @param lsb [in] 원시 LSB 데이터 배열
 * @param n [in] LSB 데이터 샘플 수 (최대 @ref NFFT)
 * @param lsb_to_ms2 [in] LSB-to-m/s^2 스케일 팩터
 * @param lsb_offset [in] 사전 보정된 오프셋(LSB, 축별) — 평균 제거 전에 반영
 * @param f_lo [in] 대역 하한 (Hz)
 * @param f_hi [in] 대역 상한 (Hz)
 * @param[out] out_rms_x100 [out] 계산된 RMS 값 (m/s^2 * 100)
 * @param[out] out_peak_x100 [out] 계산된 Peak 값 (m/s^2 * 100)
 */
static void bandlimited_rms_peak_ms2_x100(const int16_t *lsb, uint16_t n, float lsb_to_ms2, float lsb_offset, float f_lo, float f_hi, int16_t *out_rms_x100, int16_t *out_peak_x100)
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

  /* 0) DC(평균) 제거: 보정 오프셋이 설정된 경우에는 별도 평균 제거를 건너뜀 */
  const bool offset_active = (lsb_offset != 0.f);
  float mean_lsb = 0.f;
  if (!offset_active)
  {
    for (int i = 0; i < useN; ++i)
      mean_lsb += (float)lsb[i];
    mean_lsb = (useN > 0) ? (mean_lsb / useN) : 0.f;
  }

  /* 1) 윈도우 적용 및 제로 패딩 */
  for (int i = 0; i < useN; ++i)
  {
    float v = ((float)lsb[i] - lsb_offset - mean_lsb) *
              lsb_to_ms2; // 오프셋/평균 제거 후 스케일링
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

/* --- PSD 기반 가속도/속도 계산 (10–1000 Hz, N=999, Hann) ------------------ */
static int compute_psd_acc_vel_axis(const int16_t *lsb, uint16_t n,
                                    float lsb_to_ms2, float offset_lsb,
                                    int16_t *acc_rms_x100,
                                    int16_t *acc_peak_x100,
                                    int16_t *vel_rms_mmps_x100,
                                    int16_t *vel_peak_mmps_x100)
{
  if (!lsb || n == 0 || !acc_rms_x100 || !acc_peak_x100 || !vel_rms_mmps_x100 ||
      !vel_peak_mmps_x100)
    return -EINVAL;

  const uint16_t M = (n < PSD_N) ? n : PSD_N;
  if (M < PSD_K_MIN)
    return -EINVAL;

  static float win[PSD_N];
  static bool win_ready = false;
  if (!win_ready)
  {
    make_hann(win, PSD_N);
    win_ready = true;
  }

  static float re[NFFT], im[NFFT], vre[NFFT], vim[NFFT];

  /* 스케일 + 평균 제거 */
  float mean = 0.f;
  for (uint16_t i = 0; i < M; ++i)
    mean += (((float)lsb[i] - offset_lsb) * lsb_to_ms2);
  mean /= (float)M;

  for (uint16_t i = 0; i < M; ++i)
  {
    float a = (((float)lsb[i] - offset_lsb) * lsb_to_ms2) - mean;
    re[i] = a * win[i];
    im[i] = 0.f;
  }
  for (uint16_t i = M; i < NFFT; ++i)
  {
    re[i] = 0.f;
    im[i] = 0.f;
  }

  /* FFT (1024pt, zero-padding) */
  fft_radix2(re, im, 0);

  /* 원본 스펙트럼 복사 (속도 계산용) */
  memcpy(vre, re, sizeof(float) * NFFT);
  memcpy(vim, im, sizeof(float) * NFFT);

  const float df = IMU_FS_HZ / (float)PSD_N; /* 요구 스펙: 3.333... Hz */
  const int kmax = MIN(PSD_K_MAX, (NFFT / 2) - 1);

  /* Acceleration RMS (PSD 적분) */
  float acc_rms = 0.f;
  float acc_peak = 0.f;

  if (g_calc_acc)
  {
    float sum_psd_a = 0.f;
    for (int k = PSD_K_MIN; k <= kmax; ++k)
    {
      float mag2 = re[k] * re[k] + im[k] * im[k];
      float psd = 2.0f * mag2 / (PSD_HANN_U * IMU_FS_HZ * (float)PSD_N);
      sum_psd_a += psd * df;
    }
    acc_rms = sqrtf(sum_psd_a);

    /* Acceleration peak: 대역 외 제거 후 IFFT */
    for (int k = 0; k <= NFFT / 2; ++k)
    {
      bool keep = (k >= PSD_K_MIN && k <= kmax);
      if (!keep)
      {
        re[k] = 0.f;
        im[k] = 0.f;
        if (k > 0 && k < NFFT / 2)
        {
          re[NFFT - k] = 0.f;
          im[NFFT - k] = 0.f;
        }
      }
    }

    fft_radix2(re, im, 1);

    for (uint16_t i = 0; i < M; ++i)
    {
      float v = (re[i] >= 0.f) ? re[i] : -re[i];
      if (v > acc_peak)
        acc_peak = v;
    }
    acc_peak /= PSD_HANN_CG;
  }

  *acc_rms_x100 = g_calc_acc ? MS2_X100(acc_rms) : 0;
  *acc_peak_x100 = g_calc_acc ? MS2_X100(acc_peak) : 0;

  /* Velocity spectrum from original X (tmp arrays) */
  if (g_calc_vel)
  {
    float sum_psd_v = 0.f;
    for (int k = 0; k < NFFT; ++k)
    {
      re[k] = 0.f;
      im[k] = 0.f;
    }

    for (int k = PSD_K_MIN; k <= kmax; ++k)
    {
      float freq = (float)k * df;
      float omega = 2.0f * (float)M_PI * freq;
      if (omega <= 0.f)
        continue;

      float a_re = vre[k];
      float a_im = vim[k];
      float v_re = a_im / omega; /* (a + jb)/(jω) = b/ω + j(-a/ω) */
      float v_im = -a_re / omega;

      re[k] = v_re;
      im[k] = v_im;

      /* 켤레 대칭 복원 */
      int k_conj = M - k;
      if (k_conj >= 0 && k_conj < M)
      {
        re[k_conj] = v_re;
        im[k_conj] = -v_im;
      }

      float mag2 = v_re * v_re + v_im * v_im;
      float psd = 2.0f * mag2 / (PSD_HANN_U * IMU_FS_HZ * (float)PSD_N);
      sum_psd_v += psd * df;
    }

    float vel_rms = sqrtf(sum_psd_v); /* m/s */

    /* Velocity peak: IFFT */
    fft_radix2(re, im, 1);

    float vel_peak = 0.f;
    for (uint16_t i = 0; i < M; ++i)
    {
      float v = (re[i] >= 0.f) ? re[i] : -re[i];
      if (v > vel_peak)
        vel_peak = v;
    }
    vel_peak /= PSD_HANN_CG;

    *vel_rms_mmps_x100 = MMPS_X100(vel_rms);
    *vel_peak_mmps_x100 = MMPS_X100(vel_peak);
  }
  else
  {
    *vel_rms_mmps_x100 = 0;
    *vel_peak_mmps_x100 = 0;
  }

  return 0;
}

/* 가속도 TAG 값(하위 nibble) — 대부분 0x01, 일부 리비전/설정에서 0x02일 수도있어 우선 0x01을 기본으로 하되, 청크에서 다수결로 동적으로 검출하여 사용 */
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
static inline uint8_t fifo_tag_value(uint8_t raw_tag)
{
  /* 일부 환경에서 TAG가 상위 nibble(0x20 등)에 나타나는 경우가 있어 보정 */
  uint8_t lo = raw_tag & 0x0F;
  uint8_t hi = (raw_tag >> 4) & 0x0F;
  return lo ? lo : hi;
}

static inline bool is_acc_tag(uint8_t tag, uint8_t acc_tag)
{
  /* ? ????? ??? TAG? 0x01/0x02 ?? 0x04/0x07? ???? ?? ?? */
  return tag == acc_tag || tag == 0x01 || tag == 0x02 || tag == 0x04 || tag == 0x07;
}

static uint8_t detect_acc_tag(const uint8_t *buf, size_t len)
{
  int c1 = 0, c2 = 0;
  /* FIFO 선두 구간(최대 64B)에서 다수결로 TAG 검출 */
  size_t probe = MIN(len, (size_t)64);
  if (probe <= FIFO_TAG_OFFSET)
    return 0x01;
  for (size_t i = 0; i + FIFO_TAG_OFFSET < probe; ++i)
  {
    uint8_t t = fifo_tag_value(buf[i + FIFO_TAG_OFFSET]);
    if (t == 0x01)
      c1++;
    else if (t == 0x02)
      c2++;
  }
  return (c2 > c1) ? 0x02 : 0x01; /* 동률이거나 0x01이 우세하면 0x01 선택 */
}

/* 7바이트 간격으로 TAG가 반복되는 시작 오프셋 찾기 (간단 휴리스틱) */
static size_t find_sync_7B(const uint8_t *buf, size_t len, uint8_t acc_tag)
{
  size_t limit = MIN(len, (size_t)56);
  if (limit <= FIFO_TAG_OFFSET)
    return 0;
  for (size_t base = 0; base < 7 && base + 14 <= limit; ++base)
  {
    size_t ok = 0;
    for (size_t i = base; i + FIFO_TAG_OFFSET < len; i += FIFO_BYTES_PER_WORD)
    {
      uint8_t t = fifo_tag_value(buf[i + FIFO_TAG_OFFSET]);
      if (is_acc_tag(t, acc_tag))
        ok++;
      else
        break;
    }
    if (ok >= 2)
      return base;
  }
  return 0;
}

static void fifo_debug_dump_config(const char *tag)
{
  uint8_t c1 = 0, c3 = 0, c4 = 0, c5 = 0;
  uint8_t who = 0;

  rd_u8(REG_WHO_AM_I, &who);
  rd_u8(REG_CTRL1_XL, &c1);
  rd_u8(REG_FIFO_CTRL3, &c3);
  rd_u8(REG_FIFO_CTRL4, &c4);
  rd_u8(REG_FIFO_CTRL5, &c5);

  LOG_INF("FIFOcap[%s] WHO_AM_I=0x%02X, CTRL1_XL=0x%02X, FIFO_CTRL3=0x%02X, "
          "FIFO_CTRL4=0x%02X, FIFO_CTRL5=0x%02X",
          tag ? tag : "", who, c1, c3, c4, c5);
}

/* 가속도 DRDY + OUTX/Y/Z 한 샘플 디버그 */
static void lsm6dso_debug_one_sample(const char *tag)
{
  uint8_t st = 0;
  uint8_t buf[6] = {0};

  int rc1 = rd_u8(REG_STATUS_REG, &st);
  int rc2 = rd_block(REG_OUTX_L_A, buf, sizeof(buf));

  if (rc1 || rc2)
  {
    LOG_ERR("DBG_SAMPLE[%s] rd err: st=%d, xyz=%d", tag ? tag : "", rc1, rc2);
    return;
  }

  int16_t x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
  int16_t y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
  int16_t z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

  LOG_INF("DBG_SAMPLE[%s] STATUS=0x%02X (XLDA=%d), RAW X=%d, Y=%d, Z=%d",
          tag ? tag : "", st, (st & XLDA_BIT) ? 1 : 0, x, y, z);
}

/**
 * @brief 3.33kHz ODR에서 DRDY 폴링을 사용하여 가속도 데이터를 캡처하고 통계를 계산합니다.
 *
 * @details
 * 이 함수는 FIFO를 BYPASS 모드로 직접 설정하여 DRDY(Data Ready) 폴링 방식을 사용합니다.
 *
 * 1. FIFO를 BYPASS 모드로 설정합니다.
 * 2. 약 160ms 동안 또는 최대 500개 샘플을 수집할 때까지
 * REG_STATUS_REG (0x1E)의 XLDA 비트(BIT 0)를 폴링(polling)합니다.
 * 3. 새 데이터(XLDA=1)가 준비되면 REG_OUTX_L_A (0x28)부터 6바이트를 읽어 g_ax, g_ay, g_az 전역 버퍼에 저장합니다.
 * 4. 캡처가 완료되면 전체 대역(Broadband)의 RMS/Peak (m/s^2)를 계산합니다.
 * 5. bandlimited_rms_peak_ms2_x100 를 호출하여
 * 10-1000 Hz 대역의 RMS/Peak (m/s^2)를 계산합니다.
 * 6. 모든 결과를 lsm6dso_stats_t 구조체에 채웁니다.
 *
 * @param[out] out 통계 결과를 저장할 lsm6dso_stats_t 구조체 포인터
 * @param scale 캡처 시 사용할 가속도 풀스케일(LSM6DSO_SCALE_4G 또는 LSM6DSO_SCALE_16G)
 * @return 0 on success (최소 1개 샘플 수집), -EINVAL if out is NULL, -EIO if no samples collected, or I2C 에러 코드.
 */

#if 1
int lsm6dso_capture_once(lsm6dso_stats_t *out, lsm6dso_scale_t lsm6dso_scale)
{
  if (!out)
    return -EINVAL;
  memset(out, 0, sizeof(*out));

  const bool use_16g = (lsm6dso_scale == LSM6DSO_SCALE_16G);
  const float lsb_to_ms2 = use_16g ? SENS_LSB_TO_MS2_16G : SENS_LSB_TO_MS2_4G;

  fifo_debug_dump_config("DBG0_BEFORE");
  lsm6dso_debug_one_sample("BEFORE_CAP");
  rd_u8(REG_WHO_AM_I, &out->whoami);

  RC(lsm6dso_set_fs(lsm6dso_scale));
  RC(lsm6dso_apply_fifo_base());

  uint16_t total_n = 0;
  bool wtm_any = false;

  for (int rep = 0; rep < FIFO_CAPTURE_REPEAT && total_n < FIFO_TOTAL_WORDS; ++rep)
  {
    LOG_INF("FIFOcap[%d] reset: BYPASS ??", rep);

    RC(fifo_set_mode(FIFO_MODE_BYPASS));
    k_sleep(K_MSEC(2));

    RC(fifo_expect_ctrl5(NULL, ODR_FIFO_3k33_SH));
    LOG_INF("FIFOcap[%d] BYPASS OK, CTRL5=0x48 ??? ??", rep);

    RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(FIFO_WTM_WORDS & 0xFF)));
    RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((FIFO_WTM_WORDS >> 8) & 0x0F)));
    LOG_INF("FIFOcap[%d] WTM=%u words ??", rep, FIFO_WTM_WORDS);

    RC(wr_u8(REG_FIFO_CTRL4, STOP_ON_WTM_BIT));
    LOG_INF("FIFOcap[%d] STOP_ON_WTM=1 ?? (overwrite ??)", rep);

    RC(fifo_set_mode(FIFO_MODE_FIFO));
    RC(fifo_expect_ctrl5(NULL, ODR_FIFO_3k33_SH));
    LOG_INF("FIFOcap[%d] FIFO_MODE=FIFO ?? (ODR_FIFO=3.33k)", rep);

    k_sleep(K_MSEC(160));

    uint16_t diff_w = 0;
    bool wtm = false;
    for (int tries = 0; tries < 100; ++tries)
    {
      uint8_t st[2] = {0};
      int rc = rd_block(REG_FIFO_STATUS1, st, sizeof(st));
      if (rc)
      {
        LOG_ERR("FIFOcap[%d] rd FIFO_STATUS rc=%d", rep, rc);
        return rc;
      }

      diff_w = (uint16_t)(((uint16_t)(st[1] & 0x0F) << 8) | st[0]);
      wtm = (st[1] & BIT(7)) != 0;

      if (diff_w >= FIFO_WTM_WORDS)
        break;

      k_sleep(K_MSEC(2));
    }

    bool wtm_reached_now = (diff_w >= FIFO_WTM_WORDS) || wtm;
    wtm_any |= wtm_reached_now;
    LOG_INF("FIFOcap[%d] DIFF_FIFO=%u words, WTM_REACHED=%d (?? ANY=%d)",
            rep, diff_w, wtm_reached_now ? 1 : 0, wtm_any ? 1 : 0);

    if (diff_w == 0)
    {
      LOG_WRN("FIFOcap[%d]: DIFF_FIFO=0, FIFO empty", rep);
      fifo_debug_dump_config("DBG2_DIFF0");
      lsm6dso_debug_one_sample("AFTER_DIFF0");

      uint8_t fifo_dbg[21] = {0};
      uint8_t reg = REG_FIFO_DATA_OUT_TAG;
      int rc_dbg = i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, fifo_dbg, sizeof(fifo_dbg));
      if (rc_dbg)
      {
        LOG_ERR("FIFOcap[DBG_FIFO] i2c_write_read rc=%d", rc_dbg);
      }
      else
      {
        LOG_INF("FIFOcap[DBG_FIFO] first 21B: "
                "%02X %02X %02X %02X %02X %02X %02X "
                "%02X %02X %02X %02X %02X %02X %02X "
                "%02X %02X %02X %02X %02X %02X %02X",
                fifo_dbg[0], fifo_dbg[1], fifo_dbg[2], fifo_dbg[3], fifo_dbg[4],
                fifo_dbg[5], fifo_dbg[6], fifo_dbg[7], fifo_dbg[8], fifo_dbg[9],
                fifo_dbg[10], fifo_dbg[11], fifo_dbg[12], fifo_dbg[13],
                fifo_dbg[14], fifo_dbg[15], fifo_dbg[16], fifo_dbg[17],
                fifo_dbg[18], fifo_dbg[19], fifo_dbg[20]);
      }
      continue;
    }

    uint16_t words_to_read = diff_w;
    /* 더 많은 accel을 확보하기 위해 한 번에 최대 FIFO_TOTAL_WORDS까지 읽도록 허용 */
    if (words_to_read > FIFO_TOTAL_WORDS)
      words_to_read = FIFO_TOTAL_WORDS;

    uint32_t bytes_req = (uint32_t)words_to_read * FIFO_BYTES_PER_WORD;
    if (bytes_req > sizeof(g_fifo_raw))
      bytes_req = sizeof(g_fifo_raw);

    uint16_t bytes_now = (uint16_t)(bytes_req - (bytes_req % FIFO_BYTES_PER_WORD));
    if (bytes_now == 0)
    {
      LOG_WRN("FIFOcap[%d]: bytes_now=0 (DIFF_FIFO=%u)", rep, diff_w);
      continue;
    }

    LOG_INF("FIFOcap[%d] FIFO burst read: words=%u, bytes=%u",
            rep, (bytes_now / FIFO_BYTES_PER_WORD), bytes_now);

    uint8_t start_reg = REG_FIFO_DATA_OUT_TAG;
    int rc = i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &start_reg, 1, g_fifo_raw, bytes_now);
    if (rc)
    {
      LOG_ERR("FIFOcap[%d] i2c_write_read rc=%d", rep, rc);
      return rc;
    }

    uint8_t acc_tag = detect_acc_tag(g_fifo_raw, bytes_now);
    size_t sync_off = find_sync_7B(g_fifo_raw, bytes_now, acc_tag);
    LOG_INF("FIFOcap[%d] acc_tag=0x%02X, sync_off=%u", rep, acc_tag, (unsigned)sync_off);

    uint16_t n = 0;
    uint16_t pkt_total = 0;
    uint16_t pkt_skipped = 0;
    uint16_t tag_hist[16] = {0};

    LOG_INF("FIFOcap[DBG_PARSE] First 28B:");
    const size_t dbg_bytes = FIFO_BYTES_PER_WORD * 4;
    for (size_t dbg_i = 0; dbg_i < dbg_bytes && dbg_i < bytes_now;
         dbg_i += FIFO_BYTES_PER_WORD)
    {
      uint8_t raw_tag = g_fifo_raw[dbg_i + FIFO_TAG_OFFSET];
      LOG_INF("  [%02u] TAG=0x%02X(val=%u) DATA: %02X %02X %02X %02X %02X %02X",
              (unsigned)(dbg_i / 7), raw_tag, fifo_tag_value(raw_tag),
              g_fifo_raw[dbg_i + 1], g_fifo_raw[dbg_i + 2], g_fifo_raw[dbg_i + 3],
              g_fifo_raw[dbg_i + 4], g_fifo_raw[dbg_i + 5], g_fifo_raw[dbg_i + 6]);
    }

    uint16_t base_idx = total_n;
    for (size_t i = sync_off;
         i + FIFO_BYTES_PER_WORD <= bytes_now &&
         (base_idx + n) < FIFO_TOTAL_WORDS;
         i += FIFO_BYTES_PER_WORD)
    {
      uint8_t raw_tag = g_fifo_raw[i + FIFO_TAG_OFFSET];
      uint8_t tag = fifo_tag_value(raw_tag);
      if (tag < ARRAY_SIZE(tag_hist))
        tag_hist[tag]++;
      pkt_total++;

      if (n < 5)
      {
        LOG_INF("FIFOcap[DBG_TAG] rep=%d n=%u, i=%u, tag=0x%02X (full=0x%02X), acc_tag=0x%02X, match=%d",
                rep, n, (unsigned)i, tag, raw_tag, acc_tag, (tag == acc_tag) ? 1 : 0);
      }

      if (!is_acc_tag(tag, acc_tag))
      {
        pkt_skipped++;
        continue;
      }

      int16_t x = (int16_t)((uint16_t)g_fifo_raw[i + 1] | ((uint16_t)g_fifo_raw[i + 2] << 8));
      int16_t y = (int16_t)((uint16_t)g_fifo_raw[i + 3] | ((uint16_t)g_fifo_raw[i + 4] << 8));
      int16_t z = (int16_t)((uint16_t)g_fifo_raw[i + 5] | ((uint16_t)g_fifo_raw[i + 6] << 8));

      g_ax[base_idx + n] = x;
      g_ay[base_idx + n] = y;
      g_az[base_idx + n] = z;
      ++n;
    }

    total_n += n;
    LOG_INF("FIFOcap[%d] parsed accel samples: n=%u (per-run ??=%u, ??=%u)",
            rep, n, FIFO_WTM_WORDS, total_n);
    LOG_INF("FIFOcap[%d] pkt_total=%u, pkt_skipped(non-acc)=%u, tag_hist: 0x0=%u 0x1=%u 0x2=%u 0xF=%u",
            rep, pkt_total, pkt_skipped, tag_hist[0], tag_hist[1], tag_hist[2], tag_hist[0xF]);

    if (n == 0)
    {
      LOG_WRN("FIFOcap: rep %d parsed 0 accel samples", rep);
      continue;
    }
  }

  out->n = total_n;
  out->wtm_reached = wtm_any;
  if (total_n == 0)
  {
    LOG_WRN("FIFOcap: ??? ??? ??? ???? ?? (total)");
    return -EIO;
  }

  const float scale_factor = lsb_to_ms2;
  const uint16_t n = total_n;

  float mx = 0.f, my = 0.f, mz = 0.f;
  const bool offset_x_active = (g_cal_offset_lsb[0] != 0.f);
  const bool offset_y_active = (g_cal_offset_lsb[1] != 0.f);
  const bool offset_z_active = (g_cal_offset_lsb[2] != 0.f);

  if (!offset_x_active)
  {
    for (uint16_t i = 0; i < n; ++i)
      mx += (float)g_ax[i];
    mx /= (float)n;
  }
  if (!offset_y_active)
  {
    for (uint16_t i = 0; i < n; ++i)
      my += (float)g_ay[i];
    my /= (float)n;
  }
  if (!offset_z_active)
  {
    for (uint16_t i = 0; i < n; ++i)
      mz += (float)g_az[i];
    mz /= (float)n;
  }

  float sx = 0.f, sy = 0.f, sz = 0.f;
  float px = 0.f, py = 0.f, pz = 0.f;

  for (uint16_t i = 0; i < n; ++i)
  {
    float x = ((float)g_ax[i] - g_cal_offset_lsb[0] - mx) * scale_factor;
    float y = ((float)g_ay[i] - g_cal_offset_lsb[1] - my) * scale_factor;
    float z = ((float)g_az[i] - g_cal_offset_lsb[2] - mz) * scale_factor;

    float ax = fabsf(x);
    float ay = fabsf(y);
    float az = fabsf(z);

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

  float rx = sqrtf(sx / (float)n);
  float ry = sqrtf(sy / (float)n);
  float rz = sqrtf(sz / (float)n);

  out->peak_ms2_x100[0] = (int16_t)(px * 100.f + 0.5f);
  out->peak_ms2_x100[1] = (int16_t)(py * 100.f + 0.5f);
  out->peak_ms2_x100[2] = (int16_t)(pz * 100.f + 0.5f);
  out->rms_ms2_x100[0] = (int16_t)(rx * 100.f + 0.5f);
  out->rms_ms2_x100[1] = (int16_t)(ry * 100.f + 0.5f);
  out->rms_ms2_x100[2] = (int16_t)(rz * 100.f + 0.5f);

  for (int axis = 0; axis < 3; ++axis)
  {
    const int16_t *src = (axis == 0) ? g_ax : (axis == 1) ? g_ay : g_az;

    compute_psd_acc_vel_axis(
        src, n, lsb_to_ms2, g_cal_offset_lsb[axis], &out->bl_rms_ms2_x100[axis],
        &out->bl_peak_ms2_x100[axis], &out->bl_rms_mmps_x100[axis],
        &out->bl_peak_mmps_x100[axis]);
  }

  return 0;
}

#else
int lsm6dso_capture_once(lsm6dso_stats_t *out, lsm6dso_scale_t lsm6dso_scale)
{
  if (!out)
    return -EINVAL;
  memset(out, 0, sizeof(*out));

  const bool use_16g = (lsm6dso_scale == LSM6DSO_SCALE_16G);
  const uint8_t fs_bits = use_16g ? FS_XL_16G : FS_XL_4G;
  const float lsb_to_ms2 = use_16g ? SENS_LSB_TO_MS2_16G : SENS_LSB_TO_MS2_4G;

  /* -------------------------------------------------
   * 0) 가속도 설정 재확인 + FIFO 완전 비활성화
   * ------------------------------------------------- */
  /* BDU + Auto-increment */
  RC(wr_u8(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC));

  /* I3C 비활성화 (필요 시) */
  RC(wr_u8(REG_CTRL9_XL, CTRL9_XL_I3C_DISABLE));

  /* 가속도: ODR=3.33kHz, FS=±4g/±16g (CTRL1_ODR_3k33 | FS bits) */
  RC(wr_u8(REG_CTRL1_XL, (uint8_t)(CTRL1_ODR_3k33 | fs_bits)));

  /* FIFO 관련 레지스터 클리어 (배치/모드/WTM 모두 OFF) */
  RC(wr_u8(REG_FIFO_CTRL1, 0x00));
  RC(wr_u8(REG_FIFO_CTRL2, 0x00));
  RC(wr_u8(REG_FIFO_CTRL3, 0x00));
  RC(wr_u8(REG_FIFO_CTRL4, 0x00));
  /* 혹시나 해서 CTRL5도 BYPASS 모드로 */
  RC(fifo_set_mode(FIFO_MODE_BYPASS));

  /* 디버그용 WHO */
  uint8_t who = 0;
  if (rd_u8(REG_WHO_AM_I, &who) == 0)
    out->whoami = who;

  /* -------------------------------------------------
   * 1) DRDY 폴링으로 N=FIFO_WTM_WORDS 샘플 캡처
   * ------------------------------------------------- */
  const uint16_t TARGET_N = FIFO_WTM_WORDS;
  uint16_t n = 0;
  uint8_t raw[6]; /* X/Y/Z (L/H) */

  uint32_t t_start = k_uptime_get_32();
  const uint32_t CAPTURE_TIMEOUT_MS =
      1000;                             /* 1초 타임아웃 (0.3s면 충분히 끝나야 함) */
  const uint32_t POLL_INTERVAL_US = 50; /* DRDY 폴링 간격 (50us) */

  LOG_INF("DRDYcap: start capture, TARGET_N=%u", TARGET_N);

  while (n < TARGET_N)
  {
    /* 타임아웃 체크 */
    uint32_t dt = k_uptime_get_32() - t_start;
    if (dt > CAPTURE_TIMEOUT_MS)
    {
      LOG_WRN("DRDYcap: timeout, got %u/%u samples (%.1f ms)", n, TARGET_N,
              (float)dt);
      break;
    }

    uint8_t status = 0;
    RC(rd_u8(REG_STATUS_REG, &status));

    if (status & XLDA_BIT)
    {
      /* 새 샘플 준비됨 → OUTX_L_A ~ OUTZ_H_A 한 번에 읽기 */
      uint8_t reg = REG_OUTX_L_A;
      int rc =
          i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, raw, sizeof(raw));
      if (rc)
      {
        LOG_ERR("DRDYcap: i2c_write_read rc=%d (n=%u)", rc, n);
        return rc;
      }

      int16_t x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
      int16_t y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
      int16_t z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

      g_ax[n] = x;
      g_ay[n] = y;
      g_az[n] = z;
      ++n;
    }
    else
    {
      /* DRDY 아직 안 떴으면 짧게 대기 */
      k_busy_wait(POLL_INTERVAL_US);
    }
  }

  out->n = n;

  if (n == 0)
  {
    LOG_WRN("DRDYcap: no samples captured");
    return -EIO; /* rc=-5와 동일 계열 에러 */
  }

  LOG_INF("DRDYcap: captured %u samples (TARGET=%u)", n, TARGET_N);

  /* -------------------------------------------------
   * 2) DC 제거 + 전체 대역 RMS/Peak 계산
   * ------------------------------------------------- */
  const float scale_factor = lsb_to_ms2;

  /* 축별 평균 (DC 컴포넌트) 계산 */
  float mx = 0.f, my = 0.f, mz = 0.f;
  const bool offset_x_active = (g_cal_offset_lsb[0] != 0.f);
  const bool offset_y_active = (g_cal_offset_lsb[1] != 0.f);
  const bool offset_z_active = (g_cal_offset_lsb[2] != 0.f);

  if (!offset_x_active)
  {
    for (uint16_t i = 0; i < n; ++i)
      mx += (float)g_ax[i];
    mx /= (float)n;
  }
  if (!offset_y_active)
  {
    for (uint16_t i = 0; i < n; ++i)
      my += (float)g_ay[i];
    my /= (float)n;
  }
  if (!offset_z_active)
  {
    for (uint16_t i = 0; i < n; ++i)
      mz += (float)g_az[i];
    mz /= (float)n;
  }

  float sx = 0.f, sy = 0.f, sz = 0.f;
  float px = 0.f, py = 0.f, pz = 0.f;

  for (uint16_t i = 0; i < n; ++i)
  {
    float x = ((float)g_ax[i] - g_cal_offset_lsb[0] - mx) * scale_factor;
    float y = ((float)g_ay[i] - g_cal_offset_lsb[1] - my) * scale_factor;
    float z = ((float)g_az[i] - g_cal_offset_lsb[2] - mz) * scale_factor;

    float ax = fabsf(x);
    float ay = fabsf(y);
    float az = fabsf(z);

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

  float rx = sqrtf(sx / (float)n);
  float ry = sqrtf(sy / (float)n);
  float rz = sqrtf(sz / (float)n);

  out->peak_ms2_x100[0] = (int16_t)(px * 100.f + 0.5f);
  out->peak_ms2_x100[1] = (int16_t)(py * 100.f + 0.5f);
  out->peak_ms2_x100[2] = (int16_t)(pz * 100.f + 0.5f);
  out->rms_ms2_x100[0] = (int16_t)(rx * 100.f + 0.5f);
  out->rms_ms2_x100[1] = (int16_t)(ry * 100.f + 0.5f);
  out->rms_ms2_x100[2] = (int16_t)(rz * 100.f + 0.5f);

  /* -------------------------------------------------
   * 3) 10–1000 Hz 대역 제한 RMS/Peak (기존 PSD/필터 함수 활용)
   * ------------------------------------------------- */
  for (int axis = 0; axis < 3; ++axis)
  {
    const int16_t *src = (axis == 0) ? g_ax : (axis == 1) ? g_ay
                                                          : g_az;

    /* 10–1000 Hz PSD 기반 가속도/속도 계산 */
    compute_psd_acc_vel_axis(
        src, n, lsb_to_ms2, g_cal_offset_lsb[axis], &out->bl_rms_ms2_x100[axis],
        &out->bl_peak_ms2_x100[axis], &out->bl_rms_mmps_x100[axis],
        &out->bl_peak_mmps_x100[axis]);
  }

  return 0;
}
#endif

int lsm6dso_capture_acc_only(lsm6dso_stats_t *out, lsm6dso_scale_t scale)
{
  g_calc_acc = true;
  g_calc_vel = false;
  int rc = lsm6dso_capture_once(out, scale);
  g_calc_vel = true; /* restore default */
  return rc;
}

int lsm6dso_capture_vel_only(lsm6dso_stats_t *out, lsm6dso_scale_t scale)
{
  g_calc_acc = false;
  g_calc_vel = true;
  int rc = lsm6dso_capture_once(out, scale);
  g_calc_acc = true; /* restore default */
  return rc;
}

int set_calibration_lsm6dso(lsm6dso_scale_t scale)
{
  /* 오프셋을 초기화한 상태에서 한 번 캡처하여 DC 바이어스를 저장 */
  g_cal_offset_lsb[0] = g_cal_offset_lsb[1] = g_cal_offset_lsb[2] = 0.f;

  lsm6dso_stats_t st = {0};
  int rc = lsm6dso_capture_once(&st, scale);
  if (rc)
    return rc;
  if (st.n == 0)
    return -EIO;

  float sx = 0.f, sy = 0.f, sz = 0.f;
  for (uint16_t i = 0; i < st.n; ++i)
  {
    sx += (float)g_ax[i];
    sy += (float)g_ay[i];
    sz += (float)g_az[i];
  }

  const float inv_n = 1.0f / (float)st.n;
  g_cal_offset_lsb[0] = sx * inv_n;
  g_cal_offset_lsb[1] = sy * inv_n;
  g_cal_offset_lsb[2] = sz * inv_n;

  LOG_INF(
      "LSM6DSO calibration set: offset_lsb=(%.2f, %.2f, %.2f), n=%u, scale=%s",
      (double)g_cal_offset_lsb[0], (double)g_cal_offset_lsb[1],
      (double)g_cal_offset_lsb[2], st.n,
      (scale == LSM6DSO_SCALE_16G) ? "16g" : "4g");

  return 0;
}

void clear_calibration_lsm6dso(void)
{
  g_cal_offset_lsb[0] = g_cal_offset_lsb[1] = g_cal_offset_lsb[2] = 0.f;
  LOG_INF("LSM6DSO calibration cleared");
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

/* FIFO 덤프: STOP_ON_WTM|FIFO로 채운 뒤 bytes_req 바이트 버스트 읽기 → RAW+파싱
 * 출력 */
int lsm6dso_dump_fifo(const struct shell *shell, uint16_t bytes_req)
{
  if (bytes_req == 0)
    bytes_req = 224;             /* 디폴트 224B */
  bytes_req -= (bytes_req % 7u); /* 7의 배수로 정렬 */
  if (bytes_req < 7)
    bytes_req = 7;
  if (bytes_req > 420)
    bytes_req = 420; /* 과도한 로그 방지 (최대 60패킷) */

  /* 0) 완전 초기화: BYPASS + ODR 설정 고정 */
  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  k_sleep(K_MSEC(2));
  RC(wr_u8(REG_CTRL10_C, 0x00));                  /* TIMESTAMP_EN=0 */
  RC(wr_u8(REG_FIFO_CTRL3, 0x09));                /* XL only @ 3.33kHz */
  RC(fifo_expect_ctrl5(shell, ODR_FIFO_3k33_SH)); /* 기대: 0x48 */

  /* 1) 워터마크 설정 (word 단위) */
  uint16_t wtm_words = (bytes_req / 2) + 64; /* 224B -> 112 + 여유 */
  RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(wtm_words & 0xFF)));
  RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((wtm_words >> 8) & 0x0F)));

  /* 2) STOP_ON_WTM 설정(CTRL4) */
  RC(wr_u8(REG_FIFO_CTRL4, STOP_ON_WTM_BIT));

  /* 3) FIFO 시작 (여기가 진짜 핵심) */
  RC(fifo_set_mode(FIFO_MODE_FIFO));
  RC(fifo_expect_ctrl5(shell, ODR_FIFO_3k33_SH)); /* 기대: 0x48 */

  /* 3.5) 시작 직후 상태 한 번 더 읽어보기(진단) */
  uint8_t st12[2] = {0};
  RC(i2c_burst_read(i2c0, LSM6DSO_I2C_ADDR, REG_FIFO_STATUS1, st12, 2));
  uint16_t diff_w = ((uint16_t)(st12[1] & 0x0F) << 8) | st12[0];
  if (diff_w == 0)
  {
    uint8_t c5 = 0, c4 = 0;
    rd_u8(REG_FIFO_CTRL5, &c5);
    rd_u8(REG_FIFO_CTRL4, &c4);
    shell_print(shell, "after start: CTRL5=0x%02X, CTRL4=0x%02X, DIFF=%u", c5,
                c4, diff_w);
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
  shell_print(
      shell,
      "FIFO dump: req=%uB, got=%uB, DIFF≈%uB, ACC_TAG=0x%02X, sync_off=%u",
      (unsigned)bytes_req, (unsigned)bytes_now, (unsigned)bytes_avail, acc_tag,
      (unsigned)off);
  dump_hex_lines(shell, buf, bytes_now);

  /* 6) 패킷 파싱(최대 32 패킷) */
  uint16_t shown = 0;
  for (size_t i = off; i + FIFO_BYTES_PER_WORD <= bytes_now && shown < 32; i += FIFO_BYTES_PER_WORD)
  {
    uint8_t raw_tag = buf[i + FIFO_TAG_OFFSET];
    int16_t x = (int16_t)((uint16_t)buf[i + 1] | ((uint16_t)buf[i + 2] << 8));
    int16_t y = (int16_t)((uint16_t)buf[i + 3] | ((uint16_t)buf[i + 4] << 8));
    int16_t z = (int16_t)((uint16_t)buf[i + 5] | ((uint16_t)buf[i + 6] << 8));
    uint8_t tag = fifo_tag_value(raw_tag);
    shell_print(shell, "[%02u] X=%6d  Y=%6d  Z=%6d  TAG=0x%02X(val=%u) (LSB)",
                shown, x, y, z, raw_tag, tag);
    shown++;
  }
  shell_print(shell, "parsed=%u pkt (of %uB)", shown, bytes_now);

  /* 7) 모드 복구(원하면 Continuous) */
  /* 복구: Continuous @ 3.33kHz */
  RC(fifo_set_mode(FIFO_MODE_CONTINUOUS));
  RC(fifo_expect_ctrl5(shell, ODR_FIFO_3k33_SH)); /* 기대: 0x48 */

  return 0;
}
