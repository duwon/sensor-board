/**
 * @file lsm6dso.c
 * @brief LSM6DSO IMU 드라이버 (1024샘플/Active Polling/ISO 2954)
 * * @details
 * 요구사항 문서 "IMU 계산_샘플1024개.pdf" 반영
 * - 샘플링: 3.33kHz
 * - 데이터: 1024개 (512개 x 2회 연속 읽기)
 * - 모드: FIFO Continuous Mode + Active Polling (Busy Wait)
 * - 분석: 10~1000Hz 대역, Hann Window
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(lsm6dso, LOG_LEVEL_INF);

#define RC(expr)       \
  do                   \
  {                    \
    int __rc = (expr); \
    if (__rc)          \
      return __rc;     \
  } while (0)


#define PRINT_TIMING false

/**
 * @brief 현재 시간(밀리초) 반환
 * 
 * @return uint32_t 
 */
static inline uint32_t now_ms(void)
{
  return k_uptime_get_32();
}

/**
 * @brief 로그 타이밍 출력
 * 
 * @param tag :태그 문자열
 * @param t_start :시작 시점
 * @param t_prev :이전 시점
 */
static inline void log_timing(const char *tag, uint32_t t_start, uint32_t *t_prev)
{
  if (!PRINT_TIMING)
    return;

  uint32_t t_now = now_ms();
  LOG_INF("[TIMING] %s: +%u ms (total %u ms)", tag, (unsigned)(t_now - *t_prev), (unsigned)(t_now - t_start));
  *t_prev = t_now;
}

/* --- 1. 기본 설정 및 상수 (PDF 2~4페이지 참조) --- */

/* FIFO 설정: 512 워드 단위로 2회 읽기 = 총 1024 샘플  */
#define FIFO_WTM_WORDS 512    /**< Watermark Threshold (0x200) */
#define FIFO_SLACK_WORDS 0    /**< 여유분 없음 (정확히 512개 읽음) */
#define FIFO_BYTES_PER_WORD 7 /**< Tag(1) + X(2) + Y(2) + Z(2) */
#define FIFO_TAG_OFFSET 0

/* 캡처 버퍼: 512 * 2 = 1024 샘플 */
#define FIFO_CAPTURE_CHUNKS 2
#define FIFO_TOTAL_WORDS (FIFO_WTM_WORDS * FIFO_CAPTURE_CHUNKS) // 1024

/* DSP 상수 */
#define IMU_FS_HZ 3333.0f /**< 샘플링 속도 3333Hz */
#define NFFT 1024         /**< FFT 포인트 수 */
#define PSD_N 1024        /**< 데이터 개수 N */
#define PSD_K_MIN 3       /**< 10Hz 근처 (약 9.76Hz) */
#define PSD_K_MAX 307     /**< 1000Hz 근처 (약 999.2Hz) */
#define PSD_HANN_CG 0.5f  /**< Window Correction Gain */
#define PSD_HANN_U 1.5f   /**< Noise Bandwidth Coefficient */
#define G_CONST_MS2 9.80665f

/* 전역 버퍼 (BSS) */
static int16_t g_ax[FIFO_TOTAL_WORDS];
static int16_t g_ay[FIFO_TOTAL_WORDS];
static int16_t g_az[FIFO_TOTAL_WORDS];

/* FIFO RAW 버퍼: 1회 읽기 분량 (512 * 7 = 3584 Bytes) */
static uint8_t g_fifo_raw[FIFO_WTM_WORDS * FIFO_BYTES_PER_WORD];

static float g_cal_offset_lsb[3] = {0.f, 0.f, 0.f};
static bool g_calc_acc = true;
static bool g_calc_vel = true;

/* I2C & Register Definitions */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#define LSM6DSO_I2C_ADDR 0x6A

#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_FIFO_CTRL5 0x0B
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL9_XL 0x18
#define REG_CTRL10_C 0x19
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_FIFO_DATA_OUT_TAG 0x78

/* Bit Definitions */
#define CTRL3_C_BDU BIT(6)
#define CTRL3_C_IF_INC BIT(2)
#define CTRL9_XL_I3C_DISABLE BIT(1)

/* ODR & FS */
#define ODR_FIFO_3k33_SH ((uint8_t)(0x09 << 3))
#define CTRL1_ODR_3k33 0xA0
#define FS_XL_4G (0x2 << 2)
#define FS_XL_16G (0x1 << 2)

/* Scale Factors */
#define SENS_LSB_TO_MS2_4G 0.001196f
#define SENS_LSB_TO_MS2_16G 0.004784f

/* FIFO Control */
#define FIFO_MODE_BYPASS 0x0
#define FIFO_MODE_FIFO 0x1
#define FIFO_MODE_CONTINUOUS 0x6
/* STOP_ON_WTM=0 이어야 함 (Continuous) */
#define FIFO_CTRL4_CONTINUOUS_VAL (FIFO_MODE_CONTINUOUS & 0x07)

/* Result Conversion Macros */
#define MMPS_X100(v_mps) ((int16_t)((v_mps * 100000.0f) + ((v_mps >= 0.f) ? 0.5f : -0.5f)))
#define MS2_X100(v_ms2) ((int16_t)((v_ms2 * 100.0f) + ((v_ms2 >= 0.f) ? 0.5f : -0.5f)))

/* --- Helper Functions --- */

static int wr_u8(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  return i2c_write(i2c0, buf, sizeof(buf), LSM6DSO_I2C_ADDR);
}

static int rd_u8(uint8_t reg, uint8_t *val)
{
  return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, val, 1);
}

static int lsm6dso_set_fs(lsm6dso_scale_t scale)
{
  const bool use_16g = (scale == LSM6DSO_SCALE_16G);
  const uint8_t fs_bits = use_16g ? FS_XL_16G : FS_XL_4G;
  /* ODR 3.33kHz, FS 설정, LPF2=0 */
  return wr_u8(REG_CTRL1_XL, (uint8_t)(CTRL1_ODR_3k33 | fs_bits));
}

static int lsm6dso_apply_fifo_base(void)
{
  /* FIFO 설정
   * CTRL10: Timestamp off
   * FIFO_CTRL3: BDR_XL=3.33kHz (0x09)
   * FIFO_CTRL1/2: WTM=512 (0x200)
   * FIFO_CTRL4: STOP_ON_WTM=0 (Continuous를 위해)
   */
  RC(wr_u8(REG_CTRL10_C, 0x00));
  RC(wr_u8(REG_FIFO_CTRL3, 0x09));

  /* WTM = 512 */
  RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(FIFO_WTM_WORDS & 0xFF)));        // 0x00
  RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((FIFO_WTM_WORDS >> 8) & 0x0F))); // 0x02

  /* STOP_ON_WTM=0, Mode는 나중에 설정 */
  RC(wr_u8(REG_FIFO_CTRL4, 0x00));

  return 0;
}

static inline int fifo_set_mode(uint8_t mode)
{
  int rc;
  /* ODR_FIFO=3.33kHz (0x09<<3) */
  rc = wr_u8(REG_FIFO_CTRL5, ODR_FIFO_3k33_SH);
  if (rc)
    return rc;

  /* Mode 설정 (STOP_ON_WTM=0 유지)  */
  /* FIFO_CTRL4의 하위 3비트가 Mode */
  uint8_t ctrl4 = (mode & 0x07);
  return wr_u8(REG_FIFO_CTRL4, ctrl4);
}

/* --- Initialization --- */
int lsm6dso_init()
{
  /* 초기 설정 */
  RC(wr_u8(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC));
  RC(wr_u8(REG_CTRL9_XL, CTRL9_XL_I3C_DISABLE));
  RC(wr_u8(REG_CTRL2_G, 0x00)); // Gyro Power-down

  RC(lsm6dso_set_fs(LSM6DSO_SCALE_4G));
  RC(lsm6dso_apply_fifo_base());

  /* Reset to Bypass */
  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  k_sleep(K_MSEC(5));

  return 0;
}

/* --- FFT & DSP Implementations --- */

static float tw_re[NFFT / 2], tw_im[NFFT / 2];
static bool tw_ready = false;

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

static void fft_radix2(float *re, float *im, int inverse)
{
  twiddle_init();
  const int log2n = 10; // 1024

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
        float wi = inverse ? -tw_im[idx] : tw_im[idx];
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

static void make_hann(float *w, int n)
{
  for (int i = 0; i < n; ++i)
  {
    /* 0.5 * (1 - cos(2*pi*n/N-1)) */
    w[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
  }
}

/**
 * @brief 가속도 및 속도 DSP 계산 (ISO 규격/PDF 요구사항 반영)
 * @details
 */
static int compute_psd_acc_vel_axis(const int16_t *lsb, uint16_t n,
                                    float lsb_to_ms2, float offset_lsb,
                                    int16_t *acc_rms_x100,
                                    int16_t *acc_peak_x100,
                                    int16_t *vel_rms_mmps_x100,
                                    int16_t *vel_peak_mmps_x100)
{
  if (n != PSD_N)
    return -EINVAL; // Must be 1024

  uint32_t t_start = now_ms();
  uint32_t t_prev = t_start;

  static float win[PSD_N];
  static bool win_ready = false;
  if (!win_ready)
  {
    make_hann(win, PSD_N);
    win_ready = true;
  }

  static float re[NFFT], im[NFFT], vre[NFFT], vim[NFFT];

  /* 1. 가속도 전처리 (Scale, Mean Remove, Window) */
  float mean = 0.f;
  for (uint16_t i = 0; i < n; ++i)
    mean += (((float)lsb[i] - offset_lsb) * lsb_to_ms2);
  mean /= (float)n;

  for (uint16_t i = 0; i < n; ++i)
  {
    float a = (((float)lsb[i] - offset_lsb) * lsb_to_ms2) - mean;
    re[i] = a * win[i];
    im[i] = 0.f;
  }

  log_timing("PSD axis: mean/window", t_start, &t_prev);

  /* 2. FFT 수행 */
  fft_radix2(re, im, 0);

  log_timing("PSD axis: FFT forward", t_start, &t_prev);

  /* 속도 계산을 위해 원본 스펙트럼(대역제한 전) 복사 */
  memcpy(vre, re, sizeof(float) * NFFT);
  memcpy(vim, im, sizeof(float) * NFFT);

  const float df = IMU_FS_HZ / (float)PSD_N; // approx 3.255 Hz

  /* 3. 가속도 PSD 및 RMS */
  /* Band Mask: k < 3 or k > 307 => 0 */
  float sum_psd_a = 0.f;
  for (int k = 0; k < NFFT / 2; ++k)
  {
    bool in_band = (k >= PSD_K_MIN && k <= PSD_K_MAX);

    if (in_band && g_calc_acc)
    {
      float mag2 = re[k] * re[k] + im[k] * im[k];
      /* PSD formula: 2*|X|^2 / (U * Fs * N)  */
      float psd = 2.0f * mag2 / (PSD_HANN_U * IMU_FS_HZ * (float)PSD_N);
      sum_psd_a += psd * df;
    }
    else
    {
      /* 대역 외 0 처리 (Peak IFFT용)  */
      re[k] = im[k] = 0.f;
      if (k > 0)
      {
        re[NFFT - k] = im[NFFT - k] = 0.f;
      }
    }
  }
  float acc_rms = sqrtf(sum_psd_a);

  log_timing("PSD axis: accel PSD integrate", t_start, &t_prev);

  /* 4. 가속도 Peak (Inverse FFT) */
  float acc_peak = 0.f;
  if (g_calc_acc)
  {
    fft_radix2(re, im, 1); // IFFT
    for (uint16_t i = 0; i < n; ++i)
    {
      float v = fabsf(re[i]);
      if (v > acc_peak)
        acc_peak = v;
    }
    acc_peak /= PSD_HANN_CG; // divide by 0.5
  }

  log_timing("PSD axis: accel peak IFFT", t_start, &t_prev);

  *acc_rms_x100 = MS2_X100(acc_rms);
  *acc_peak_x100 = MS2_X100(acc_peak);

  /* 5. 속도 변환 (적분) 및 PSD */
  float sum_psd_v = 0.f;
  if (g_calc_vel)
  {
    for (int k = 0; k < NFFT; ++k)
    {
      re[k] = im[k] = 0.f; // Clear for velocity IFFT
    }

    for (int k = PSD_K_MIN; k <= PSD_K_MAX; ++k)
    {
      float freq = (float)k * df;
      float omega = 2.0f * (float)M_PI * freq;

      /* V[k] = X[k] / (j * omega) = (a+jb)*(-j/w) = (b/w) + j(-a/w) */
      float v_re = vim[k] / omega;
      float v_im = -vre[k] / omega;

      re[k] = v_re;
      im[k] = v_im;

      /* Conjugate symmetry for Real IFFT */
      re[NFFT - k] = v_re;
      im[NFFT - k] = -v_im;

      float mag2 = v_re * v_re + v_im * v_im;
      float psd = 2.0f * mag2 / (PSD_HANN_U * IMU_FS_HZ * (float)PSD_N);
      sum_psd_v += psd * df;
    }
  }
  float vel_rms = sqrtf(sum_psd_v); // m/s

  log_timing("PSD axis: velocity PSD", t_start, &t_prev);

  /* 6. 속도 Peak (Inverse FFT) */
  float vel_peak = 0.f;
  if (g_calc_vel)
  {
    fft_radix2(re, im, 1); // IFFT
    for (uint16_t i = 0; i < n; ++i)
    {
      float v = fabsf(re[i]);
      if (v > vel_peak)
        vel_peak = v;
    }
    vel_peak /= PSD_HANN_CG; // divide by 0.5
  }

  log_timing("PSD axis: velocity peak IFFT", t_start, &t_prev);

  *vel_rms_mmps_x100 = MMPS_X100(vel_rms);
  *vel_peak_mmps_x100 = MMPS_X100(vel_peak);

  return 0;
}

/**
 * @brief 1024샘플 캡처 루틴
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

int lsm6dso_capture_once(lsm6dso_stats_t *out, lsm6dso_scale_t lsm6dso_scale)
{
  if (!out)
    return -EINVAL;
  memset(out, 0, sizeof(*out));

  uint32_t t_start = now_ms();
  uint32_t t_prev = t_start;

  const bool use_16g = (lsm6dso_scale == LSM6DSO_SCALE_16G);
  const float lsb_to_ms2 = use_16g ? SENS_LSB_TO_MS2_16G : SENS_LSB_TO_MS2_4G;

  rd_u8(REG_WHO_AM_I, &out->whoami);
  log_timing("WHO_AM_I read", t_start, &t_prev);

  /* 설정 적용 */
  RC(lsm6dso_set_fs(lsm6dso_scale));
  log_timing("set_fs", t_start, &t_prev);
  RC(lsm6dso_apply_fifo_base());
  log_timing("apply_fifo_base", t_start, &t_prev);

  /* 1. FIFO 리셋 (Bypass -> Continuous) */
  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  log_timing("fifo bypass", t_start, &t_prev);
  k_busy_wait(100);

  RC(fifo_set_mode(FIFO_MODE_CONTINUOUS)); // Start Capture
  log_timing("fifo continuous start", t_start, &t_prev);

  /* 1024개 샘플 수집 (512 * 2 chunks) */
  uint16_t total_parsed = 0;

  for (int chunk = 0; chunk < 2; ++chunk)
  {
    /* 2. & 4. Polling (Busy Wait) */
    /* 조건: WTM 플래그(Bit7) == 1 or DIFF >= 512 */
    /* Timeout : 512샘플 @ 3.33k = ~154ms. 여유 200ms */
    uint32_t chunk_wait_start = now_ms();
    bool ready = false;

    while ((now_ms() - chunk_wait_start) < 250)
    {
      uint8_t st[2];
      /* i2c_burst_read를 사용하여 STATUS1,2를 한번에 읽음 */
      if (i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, (uint8_t[]){REG_FIFO_STATUS1}, 1, st, 2) != 0)
      {
        return -EIO;
      }

      uint16_t diff = ((uint16_t)(st[1] & 0x0F) << 8) | st[0];
      bool wtm = (st[1] & 0x80) != 0; // Bit 7

      if (wtm || diff >= FIFO_WTM_WORDS)
      {
        ready = true;
        break;
      }
      /* Busy Wait (Sleep 없음) */
      k_busy_wait(1);
    }

    if (!ready)
    {
      LOG_ERR("FIFO timeout at chunk %d", chunk);
      return -EIO;
    }

    log_timing(chunk == 0 ? "chunk0 wait WTM" : "chunk1 wait WTM", t_start, &t_prev);

    /* 3. & 5. Burst Read */
    /* 읽어야 할 바이트: 512 * 7 = 3584 bytes */
    uint8_t reg_addr = REG_FIFO_DATA_OUT_TAG;
    if (i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg_addr, 1, g_fifo_raw, sizeof(g_fifo_raw)) != 0)
    {
      LOG_ERR("FIFO read fail at chunk %d", chunk);
      return -EIO;
    }

    log_timing(chunk == 0 ? "chunk0 burst read" : "chunk1 burst read", t_start, &t_prev);

    /* 데이터 파싱 (Raw_Data -> g_ax/ay/az) */
    /* PDF는 Raw_Data[0~511], [512~1023] 저장. */
    /* FIFO 구조: Tag(1) + X(2) + Y(2) + Z(2) = 7 Bytes */
#if 0
    int parse_idx = 0;
    uint16_t base_idx = chunk * FIFO_WTM_WORDS; // 0 or 512

    for (int i = 0; i < sizeof(g_fifo_raw); i += FIFO_BYTES_PER_WORD)
    {
      /* Tag 확인: 가속도 데이터(0x01/0x02)인지 체크하는 것이 좋으나 연속 읽기를 통한 타이밍 보장을 위해 순서대로 저장 */
      uint8_t tag = (g_fifo_raw[i] >> 3);

      int16_t x = (int16_t)((uint16_t)g_fifo_raw[i + 1] | ((uint16_t)g_fifo_raw[i + 2] << 8));
      int16_t y = (int16_t)((uint16_t)g_fifo_raw[i + 3] | ((uint16_t)g_fifo_raw[i + 4] << 8));
      int16_t z = (int16_t)((uint16_t)g_fifo_raw[i + 5] | ((uint16_t)g_fifo_raw[i + 6] << 8));

      g_ax[base_idx + parse_idx] = x;
      g_ay[base_idx + parse_idx] = y;
      g_az[base_idx + parse_idx] = z;
      parse_idx++;
    }
    total_parsed += parse_idx;
#else
    int parse_idx = 0;
    int acc_valid_count = 0;                    // 가속도 태그 카운트 변수 추가
    uint16_t base_idx = chunk * FIFO_WTM_WORDS; // 0 or 512

    for (int i = 0; i < sizeof(g_fifo_raw); i += FIFO_BYTES_PER_WORD)
    {
      /* Tag 추출  [7:3]Sensor Tag */
      uint8_t raw_tag = g_fifo_raw[i];
      uint8_t tag_val = (raw_tag >> 3) & 0x1F;

      /* 가속도 태그 확인 (0x01: XL, 0x02: XL_NC) */
      if (tag_val == 0x01 || tag_val == 0x02)
      {
        acc_valid_count++;
      }

      int16_t x = (int16_t)((uint16_t)g_fifo_raw[i + 1] | ((uint16_t)g_fifo_raw[i + 2] << 8));
      int16_t y = (int16_t)((uint16_t)g_fifo_raw[i + 3] | ((uint16_t)g_fifo_raw[i + 4] << 8));
      int16_t z = (int16_t)((uint16_t)g_fifo_raw[i + 5] | ((uint16_t)g_fifo_raw[i + 6] << 8));

      g_ax[base_idx + parse_idx] = x;
      g_ay[base_idx + parse_idx] = y;
      g_az[base_idx + parse_idx] = z;
      parse_idx++;
    }

    LOG_INF("Chunk %d: Parsed %d samples, Valid Acc Tags: %d", chunk, parse_idx, acc_valid_count);

    /* 태그 불인치 인 경우*/
    if (acc_valid_count != parse_idx)
    {
      LOG_WRN("Chunk %d: Mixed data detected! (Non-Acc samples: %d)", chunk, parse_idx - acc_valid_count);
    }

    total_parsed += parse_idx;
    log_timing(chunk == 0 ? "chunk0 parse" : "chunk1 parse", t_start, &t_prev);
#endif
  }

  out->n = total_parsed;
  out->wtm_reached = true;

  /* 6. 종료: 센서 Power-down / I2C Off (여기서는 Bypass로 전환) */
  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  log_timing("capture complete (fifo stop)", t_start, &t_prev);

  if (total_parsed != 1024)
  {
    LOG_WRN("Sample count mismatch: %d", total_parsed);
    return -EIO;
  }

  log_timing("capture complete (samples ready)", t_start, &t_prev);

  /* --- DSP Calculation --- */

  /* 전체 대역 통계 (Raw RMS/Peak) */
  float sum_sq[3] = {0}, max_val[3] = {0};
  for (int i = 0; i < total_parsed; i++)
  {
    float vals[3];
    vals[0] = ((float)g_ax[i] - g_cal_offset_lsb[0]) * lsb_to_ms2;
    vals[1] = ((float)g_ay[i] - g_cal_offset_lsb[1]) * lsb_to_ms2;
    vals[2] = ((float)g_az[i] - g_cal_offset_lsb[2]) * lsb_to_ms2;

    for (int axis = 0; axis < 3; axis++)
    {
      float v = vals[axis];
      sum_sq[axis] += v * v;
      if (fabsf(v) > max_val[axis])
        max_val[axis] = fabsf(v);
    }
  }
  for (int i = 0; i < 3; i++)
  {
    out->peak_ms2_x100[i] = MS2_X100(max_val[i]);
    out->rms_ms2_x100[i] = MS2_X100(sqrtf(sum_sq[i] / total_parsed));
  }

  log_timing("broadband rms/peak", t_start, &t_prev);

  /* 10-1000Hz 대역 제한 통계 (FFT 기반) */
  for (int axis = 0; axis < 3; ++axis)
  {
    const int16_t *src = (axis == 0) ? g_ax : (axis == 1) ? g_ay
                                                          : g_az;
    compute_psd_acc_vel_axis(
        src, total_parsed, lsb_to_ms2, g_cal_offset_lsb[axis],
        &out->bl_rms_ms2_x100[axis], &out->bl_peak_ms2_x100[axis],
        &out->bl_rms_mmps_x100[axis], &out->bl_peak_mmps_x100[axis]);
    log_timing(axis == 0 ? "PSD axis X total" : (axis == 1 ? "PSD axis Y total" : "PSD axis Z total"), t_start, &t_prev);
  }

  log_timing("capture_once total", t_start, &t_prev);

  return 0;
}

int lsm6dso_capture_acc_only(lsm6dso_stats_t *out, lsm6dso_scale_t scale)
{
  g_calc_acc = true;
  g_calc_vel = false;
  int rc = lsm6dso_capture_once(out, scale);
  g_calc_vel = true;
  return rc;
}

int lsm6dso_capture_vel_only(lsm6dso_stats_t *out, lsm6dso_scale_t scale)
{
  g_calc_acc = false;
  g_calc_vel = true;
  int rc = lsm6dso_capture_once(out, scale);
  g_calc_acc = true;
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

int lsm6dso_dump_regs(const struct shell *shell)
{
  /* 기존 구현 유지 */
  uint8_t v;
  rd_u8(REG_WHO_AM_I, &v);
  shell_print(shell, "WHO: %02X", v);
  rd_u8(REG_FIFO_CTRL1, &v);
  shell_print(shell, "FIFO1: %02X", v);
  rd_u8(REG_FIFO_CTRL2, &v);
  shell_print(shell, "FIFO2: %02X", v);
  rd_u8(REG_FIFO_CTRL3, &v);
  shell_print(shell, "FIFO3: %02X", v);
  rd_u8(REG_FIFO_CTRL4, &v);
  shell_print(shell, "FIFO4: %02X", v);
  return 0;
}
