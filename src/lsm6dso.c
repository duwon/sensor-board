/**
 * @file lsm6dso.c
 * @brief LSM6DSO IMU 드라이버 (1024샘플/Active Polling/ISO 10816)
 *
 * @details
 * 요구사항 문서의 계산 흐름을 반영한다.
 * - 샘플링: 3.33kHz
 * - 데이터: 1024개 (256개 x 4회 연속 읽기)
 * - 모드: FIFO Continuous Mode + Active Polling (Busy Wait)
 * - 분석: 10~1000Hz 대역, Hann Window, CMSIS-DSP RFFT
 * - Peak: Equivalent Peak = True RMS * sqrt(2)
 */

#include "lsm6dso.h"
#include <arm_math.h>
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

/* --- 1. 기본 설정 및 상수 (PDF 2~4페이지 참조) --- */

/* FIFO 설정: 256 워드 단위로 4회 읽기 = 총 1024 샘플 (방법 B: WTM < FIFO 최대 용량) */
#define FIFO_WTM_WORDS 128
#define FIFO_BYTES_PER_WORD 7
#define FIFO_CAPTURE_CHUNKS 8
#define FIFO_TOTAL_WORDS (FIFO_WTM_WORDS * FIFO_CAPTURE_CHUNKS)

/* DSP 상수 */
#define FFT_SIZE 1024
#define SAMPLE_RATE_HZ 3333.0f           /* 샘플링 속도 3333Hz */
#define BAND_BIN_START 3                 /* 10Hz 근처 (약 9.76Hz) */
#define BAND_BIN_END 307                 /* 1000Hz 근처 (약 999.2Hz) */
#define HANN_RMS_CORRECTION 1.633f       /* Hann RMS/energy correction for band-RMS calculation */
#define EQ_PEAK_FACTOR 1.414213f         /* sqrt(2) */
#define G_CONST_MS2 9.80665f
#define SENSITIVITY_4G_G_PER_LSB 0.000122f  /* 4g 모드 감도 (g/LSB) */
#define SENSITIVITY_16G_G_PER_LSB 0.000488f /* 16g 모드 감도 (g/LSB) */

/* I2C & Register Definitions */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#define LSM6DSO_I2C_ADDR 0x6A

#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL9_XL 0x18
#define REG_CTRL10_C 0x19
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_FIFO_DATA_OUT_TAG 0x78

#define CTRL3_C_BDU BIT(6)
#define CTRL3_C_IF_INC BIT(2)
#define CTRL9_XL_I3C_DISABLE BIT(1)

#define CTRL1_ODR_3k33 0x90
#define FS_XL_4G (0x2 << 2)
#define FS_XL_16G (0x1 << 2)

#define FIFO_MODE_BYPASS 0x0
#define FIFO_MODE_CONTINUOUS 0x6

#define MMPS_X100(v_mps) ((int16_t)((v_mps * 100000.0f) + ((v_mps >= 0.f) ? 0.5f : -0.5f)))
#define MS2_X100(v_ms2) ((int16_t)((v_ms2 * 100.0f) + ((v_ms2 >= 0.f) ? 0.5f : -0.5f)))

BUILD_ASSERT(FIFO_TOTAL_WORDS == FFT_SIZE, "FIFO capture must match FFT size");

/* 전역 버퍼 (BSS) */
static int16_t g_ax[FIFO_TOTAL_WORDS];
static int16_t g_ay[FIFO_TOTAL_WORDS];
static int16_t g_az[FIFO_TOTAL_WORDS];

/* FIFO RAW 버퍼: 1회 읽기 분량 (256 * 7 = 1792 Bytes) */
static uint8_t g_fifo_word[FIFO_BYTES_PER_WORD];

/* CMSIS-DSP 작업 버퍼 */
static arm_rfft_fast_instance_f32 g_rfft_inst;
static bool g_rfft_ready = false;
static bool g_hann_window_ready = false;
static float g_hann_window[FFT_SIZE];
static float g_fft_buffer[FFT_SIZE];
static float g_fft_output[FFT_SIZE];
static float g_mag_buffer[FFT_SIZE / 2];

static float g_cal_offset_lsb[3] = {0.0f, 0.0f, 0.0f};
static bool g_calc_acc = true;
static bool g_calc_vel = true;

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
 * @param tag 태그 문자열
 * @param t_start 시작 시점
 * @param t_prev 이전 시점
 */
static inline void log_timing(const char *tag, uint32_t t_start, uint32_t *t_prev)
{
  if (!PRINT_TIMING)
  {
    return;
  }

  uint32_t t_now = now_ms();
  LOG_INF("[TIMING] %s: +%u ms (total %u ms)",
          tag,
          (unsigned)(t_now - *t_prev),
          (unsigned)(t_now - t_start));
  *t_prev = t_now;
}

static int wr_u8(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  return i2c_write(i2c0, buf, sizeof(buf), LSM6DSO_I2C_ADDR);
}

static int rd_u8(uint8_t reg, uint8_t *val)
{
  return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, val, 1);
}

/* --- Helper Functions --- */

static int lsm6dso_set_fs(lsm6dso_scale_t scale)
{
  const bool use_16g = (scale == LSM6DSO_SCALE_16G);
  const uint8_t fs_bits = use_16g ? FS_XL_16G : FS_XL_4G;

  return wr_u8(REG_CTRL1_XL, (uint8_t)(CTRL1_ODR_3k33 | fs_bits));
}

static int lsm6dso_apply_fifo_base(void)
{
  /* FIFO 설정
   * CTRL10: Timestamp off
   * FIFO_CTRL3: BDR_XL=3.33kHz (0x09)
   * FIFO_CTRL1/2: WTM=128 (0x080) — WTM[7:0]=0x80, WTM8=0
   *   WTM 필드는 9비트(0~511)이므로 512를 설정하면 0으로 오버플로우됨
   *   128 word 단위로 8회 읽어 1024 샘플 확보
   * FIFO_CTRL2: STOP_ON_WTM=0 (Continuous mode 유지)
   */
  RC(wr_u8(REG_CTRL10_C, 0x00));
  RC(wr_u8(REG_FIFO_CTRL3, 0x09));
  RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(FIFO_WTM_WORDS & 0xFF)));
  RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((FIFO_WTM_WORDS >> 8) & 0x01)));
  RC(wr_u8(REG_FIFO_CTRL4, 0x00));

  return 0;
}

static int fifo_set_mode(uint8_t mode)
{
  uint8_t reg = 0;

  RC(rd_u8(REG_FIFO_CTRL4, &reg));
  reg = (uint8_t)((reg & ~0x07U) | (mode & 0x07U));
  return wr_u8(REG_FIFO_CTRL4, reg);
}

static int fifo_read_word(uint8_t *word)
{
  uint8_t reg_addr = REG_FIFO_DATA_OUT_TAG;
  return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg_addr, 1,
                        word, FIFO_BYTES_PER_WORD);
}

static int lsm6dso_prepare_rfft(void)
{
  /* CMSIS-DSP RFFT 인스턴스는 1회만 초기화한다. */
  if (g_rfft_ready)
  {
    return 0;
  }

  arm_status status = arm_rfft_fast_init_f32(&g_rfft_inst, FFT_SIZE);
  if (status != ARM_MATH_SUCCESS)
  {
    LOG_ERR("arm_rfft_fast_init_f32 failed: %d", (int)status);
    return -EINVAL;
  }

  g_rfft_ready = true;
  return 0;
}

static void lsm6dso_prepare_hann_window(void)
{
  /* Hann window 계수는 정적 버퍼에 1회 생성 후 재사용한다. */
  if (g_hann_window_ready)
  {
    return;
  }

  for (int i = 0; i < FFT_SIZE; ++i)
  {
    g_hann_window[i] =
        0.5f * (1.0f - cosf((2.0f * (float)M_PI * (float)i) / (float)(FFT_SIZE - 1)));
  }

  g_hann_window_ready = true;
}

static int lsm6dso_prepare_dsp(void)
{
  /* FFT 인스턴스와 window 계수를 모두 준비한다. */
  int rc = lsm6dso_prepare_rfft();
  if (rc < 0)
  {
    return rc;
  }

  lsm6dso_prepare_hann_window();
  return 0;
}

/* --- Initialization --- */
int lsm6dso_init(void)
{
  /* 초기 설정 */
  RC(wr_u8(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC));
  RC(wr_u8(REG_CTRL9_XL, CTRL9_XL_I3C_DISABLE));
  RC(wr_u8(REG_CTRL2_G, 0x00));

  RC(lsm6dso_set_fs(LSM6DSO_SCALE_4G));
  RC(lsm6dso_apply_fifo_base());
  RC(lsm6dso_prepare_dsp());

  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  k_sleep(K_MSEC(5));

  return 0;
}

/**
 * @brief 가속도 및 속도 DSP 계산 (ISO 규격/PDF 요구사항 반영)
 *
 * @details
 * 축별로 다음 순서로 band-limited 결과를 계산한다.
 * 1. Offset 제거 및 평균 제거
 * 2. LSB -> g -> m/s^2 변환
 * 3. Hann window 적용
 * 4. CMSIS-DSP RFFT + magnitude 계산
 * 5. 10~1000Hz 대역만 사용하여 RMS 계산
 * 6. Equivalent Peak = True RMS * sqrt(2)
 */
static int compute_psd_acc_vel_axis(const int16_t *lsb, uint16_t n,
                                    float sensitivity_g_per_lsb, float offset_lsb,
                                    int16_t *acc_rms_x100,
                                    int16_t *acc_peak_x100,
                                    int16_t *vel_rms_mmps_x100,
                                    int16_t *vel_peak_mmps_x100)
{
  if (n != FFT_SIZE)
  {
    return -EINVAL;
  }

  int rc = lsm6dso_prepare_dsp();
  if (rc < 0)
  {
    return rc;
  }

  uint32_t t_start = now_ms();
  uint32_t t_prev = t_start;

  /* [2 단계] DC 성분(평균값) 제거 */
  float mean_raw = 0.0f;
  for (uint16_t i = 0; i < n; ++i)
  {
    mean_raw += ((float)lsb[i] - offset_lsb);
  }
  mean_raw /= (float)n;

  for (uint16_t i = 0; i < n; ++i)
  {
    /* [3 단계] 가속도 단위 변환 [m/s^2] */
    float raw_zeroed = ((float)lsb[i] - offset_lsb) - mean_raw;
    float accel_g = raw_zeroed * sensitivity_g_per_lsb;
    float accel_ms2 = accel_g * G_CONST_MS2;

    /* [4 단계] Hanning Window 적용 */
    g_fft_buffer[i] = accel_ms2 * g_hann_window[i];
  }

  log_timing("Band axis: mean/window", t_start, &t_prev);

  /* [5 단계] FFT 수행 및 Magnitude 도출 */
  arm_rfft_fast_f32(&g_rfft_inst, g_fft_buffer, g_fft_output, 0);
  log_timing("Band axis: RFFT", t_start, &t_prev);

  arm_cmplx_mag_f32(g_fft_output, g_mag_buffer, FFT_SIZE / 2);
  log_timing("Band axis: magnitude", t_start, &t_prev);

  /* [6 단계] 10~1000Hz 대역 인덱스 설정은 BAND_BIN_START/BAND_BIN_END 매크로 사용 */
  const float df = SAMPLE_RATE_HZ / (float)FFT_SIZE;
  float sum_accel_sq = 0.0f;
  float sum_vel_sq = 0.0f;

  for (int k = BAND_BIN_START; k <= BAND_BIN_END; ++k)
  {
    float freq = (float)k * df;
    /* RMS 계산을 위한 Hann 에너지 보정 */
    float ak = (g_mag_buffer[k] / (FFT_SIZE / 2.0f)) * HANN_RMS_CORRECTION;

    if (g_calc_acc)
    {
      /* [7 단계] 가속도 에너지 누적 */
      sum_accel_sq += ak * ak;
    }

    if (g_calc_vel)
    {
      /* [10 단계] 주파수별 속도 변환 [mm/s] */
      float vk = (ak / (2.0f * (float)M_PI * freq)) * 1000.0f;

      /* [11 단계] 속도 에너지 누적 */
      sum_vel_sq += vk * vk;
    }
  }

  log_timing("Band axis: band sum", t_start, &t_prev);

  if (g_calc_acc)
  {
    /* [8 단계] 가속도 True RMS 계산 */
    float acc_rms = sqrtf(sum_accel_sq / 2.0f);

    /* [9 단계] 가속도 Equivalent Peak 계산 */
    float acc_eq_peak = acc_rms * EQ_PEAK_FACTOR;
    *acc_rms_x100 = MS2_X100(acc_rms);
    *acc_peak_x100 = MS2_X100(acc_eq_peak);
  }
  else
  {
    *acc_rms_x100 = 0;
    *acc_peak_x100 = 0;
  }

  if (g_calc_vel)
  {
    /* [12 단계] 속도 True RMS 계산 */
    float vel_rms = sqrtf(sum_vel_sq / 2.0f);

    /* [13 단계] 속도 Equivalent Peak 계산 */
    float vel_eq_peak = vel_rms * EQ_PEAK_FACTOR;
    *vel_rms_mmps_x100 = MMPS_X100(vel_rms);
    *vel_peak_mmps_x100 = MMPS_X100(vel_eq_peak);
  }
  else
  {
    *vel_rms_mmps_x100 = 0;
    *vel_peak_mmps_x100 = 0;
  }

  log_timing("Band axis: rms/peak", t_start, &t_prev);
  return 0;
}

/**
 * @brief 1024샘플 캡처 루틴
 *
 * @details
 * FIFO를 BYPASS -> CONTINUOUS로 전환한 뒤 Active Polling으로
 * 128샘플씩 8회 읽어서 총 1024샘플을 수집한다.
 *
 * 1. WHO_AM_I, FS, FIFO 기본 설정을 적용한다.
 * 2. FIFO watermark(WTM_IA) 또는 DIFF_FIFO >= WTM 조건을 Busy Wait으로 감시한다.
 * 3. REG_FIFO_DATA_OUT_TAG부터 256워드씩 4회 burst read 한다.
 * 4. 가속도 태그(0x02=Accelerometer NC)만 허용하고, mixed data는 에러로 종료한다.
 * 5. 전체 대역(Broadband)의 RMS/Peak를 시간영역 기준으로 계산한다.
 * 6. 10~1000Hz 대역의 RMS/Equivalent Peak를 PDF 계산식으로 산출한다.
 *
 * @param[out] out 통계 결과를 저장할 lsm6dso_stats_t 구조체 포인터
 * @param scale 캡처 시 사용할 가속도 풀스케일
 * @return 0 on success, -EINVAL if out is NULL, -EIO on capture/parsing failure
 */
int lsm6dso_capture_once(lsm6dso_stats_t *out, lsm6dso_scale_t scale)
{
  if (!out)
  {
    return -EINVAL;
  }

  memset(out, 0, sizeof(*out));

  uint32_t t_start = now_ms();
  uint32_t t_prev = t_start;

  const bool use_16g = (scale == LSM6DSO_SCALE_16G);
  const float sensitivity_g_per_lsb =
      use_16g ? SENSITIVITY_16G_G_PER_LSB : SENSITIVITY_4G_G_PER_LSB;
  const float lsb_to_ms2 = sensitivity_g_per_lsb * G_CONST_MS2;

  (void)rd_u8(REG_WHO_AM_I, &out->whoami);
  log_timing("WHO_AM_I read", t_start, &t_prev);

  RC(lsm6dso_set_fs(scale));
  log_timing("set_fs", t_start, &t_prev);

  RC(lsm6dso_apply_fifo_base());
  log_timing("apply_fifo_base", t_start, &t_prev);

  RC(lsm6dso_prepare_dsp());
  log_timing("prepare_dsp", t_start, &t_prev);

  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  log_timing("fifo bypass", t_start, &t_prev);
  k_busy_wait(100);

  RC(fifo_set_mode(FIFO_MODE_CONTINUOUS));
  log_timing("fifo continuous start", t_start, &t_prev);

  /* 1024개 샘플 수집 (128 * 8 chunks, WTM=128) */
  uint16_t total_parsed = 0;

  for (int chunk = 0; chunk < FIFO_CAPTURE_CHUNKS; ++chunk)
  {
    uint32_t chunk_wait_start = now_ms();
    bool ready = false;

    /* 조건: WTM 플래그(FIFO_STATUS2 bit7) == 1 or DIFF >= WTM_WORDS
     * Timeout : 128샘플 @ 3.33kHz = 약 39ms, 여유 포함 250ms
     * DIFF_FIFO[9:8]는 FIFO_STATUS2 bit[1:0]에 위치 → 마스크 0x03
     */
    while ((now_ms() - chunk_wait_start) < 250)
    {
      uint8_t st[2];
      if (i2c_write_read(i2c0, LSM6DSO_I2C_ADDR,
                         (uint8_t[]){REG_FIFO_STATUS1}, 1, st, 2) != 0)
      {
        return -EIO;
      }

      /* FIFO_STATUS2 비트 검사 (DS Table 113)
       * bit7=WTM_IA, bit6=OVR_IA, bit5=FULL_IA, bit3=OVR_LATCHED,
       * bit[1:0]=DIFF_FIFO[9:8] */
      if (st[1] & 0x48) /* OVR_IA(bit6) | OVR_LATCHED(bit3) */
      {
        LOG_ERR("FIFO overflow at chunk %d (STATUS2=0x%02x)", chunk, st[1]);
        return -EIO;
      }
      uint16_t diff = ((uint16_t)(st[1] & 0x03) << 8) | st[0];
      bool wtm = (st[1] & 0x80) != 0;
      if (wtm || diff >= FIFO_WTM_WORDS)
      {
        ready = true;
        break;
      }

      k_busy_wait(1);
    }

    if (!ready)
    {
      LOG_ERR("FIFO timeout at chunk %d", chunk);
      return -EIO;
    }

    log_timing("chunk wait WTM", t_start, &t_prev);

    /* FIFO 구조: Tag(1) + X(2) + Y(2) + Z(2) = 7 Bytes */
    uint16_t base_idx = (uint16_t)(chunk * FIFO_WTM_WORDS);
    uint16_t acc_valid_count = 0;
    uint16_t non_acc_count = 0;

    for (uint16_t word = 0; word < FIFO_WTM_WORDS; ++word)
    {
      if (fifo_read_word(g_fifo_word) != 0)
      {
        LOG_ERR("FIFO read fail at chunk %d word %u",
                chunk, (unsigned)word);
        return -EIO;
      }
      /* TAG_SENSOR[4:0] 추출 (DS Table 165): 0x02 = Accelerometer NC
       * 0x01은 Gyroscope NC이므로 허용하지 않는다 */
      uint8_t tag_val = (uint8_t)((g_fifo_word[0] >> 3) & 0x1F);
      if (tag_val != 0x02)
      {
        non_acc_count++;
        continue;
      }

      if (acc_valid_count >= FIFO_WTM_WORDS)
      {
        LOG_ERR("Chunk %d produced too many accel samples", chunk);
        return -EIO;
      }

      int16_t x = (int16_t)((uint16_t)g_fifo_word[1] |
                            ((uint16_t)g_fifo_word[2] << 8));
      int16_t y = (int16_t)((uint16_t)g_fifo_word[3] |
                            ((uint16_t)g_fifo_word[4] << 8));
      int16_t z = (int16_t)((uint16_t)g_fifo_word[5] |
                            ((uint16_t)g_fifo_word[6] << 8));

      g_ax[base_idx + acc_valid_count] = x;
      g_ay[base_idx + acc_valid_count] = y;
      g_az[base_idx + acc_valid_count] = z;
      acc_valid_count++;
    }

    if (non_acc_count != 0)
    {
      LOG_ERR("Chunk %d contains %u non-accelerometer samples",
              chunk, (unsigned)non_acc_count);
      return -EIO;
    }

    if (acc_valid_count != FIFO_WTM_WORDS)
    {
      LOG_ERR("Chunk %d accel sample count mismatch: %u",
              chunk, (unsigned)acc_valid_count);
      return -EIO;
    }

    total_parsed += acc_valid_count;
    log_timing("chunk read/parse", t_start, &t_prev);
  }

  out->n = total_parsed;
  out->wtm_reached = true;

  RC(fifo_set_mode(FIFO_MODE_BYPASS));
  log_timing("capture complete (fifo stop)", t_start, &t_prev);

  if (total_parsed != FFT_SIZE)
  {
    LOG_ERR("Sample count mismatch: %u", (unsigned)total_parsed);
    return -EIO;
  }

  log_timing("capture complete (samples ready)", t_start, &t_prev);

  /* 전체 대역 통계 (Raw RMS/Peak) */
  float sum_sq[3] = {0.0f, 0.0f, 0.0f};
  float max_val[3] = {0.0f, 0.0f, 0.0f};

  for (uint16_t i = 0; i < total_parsed; ++i)
  {
    float vals[3];
    vals[0] = ((float)g_ax[i] - g_cal_offset_lsb[0]) * lsb_to_ms2;
    vals[1] = ((float)g_ay[i] - g_cal_offset_lsb[1]) * lsb_to_ms2;
    vals[2] = ((float)g_az[i] - g_cal_offset_lsb[2]) * lsb_to_ms2;

    for (int axis = 0; axis < 3; ++axis)
    {
      float v = vals[axis];
      sum_sq[axis] += v * v;
      if (fabsf(v) > max_val[axis])
      {
        max_val[axis] = fabsf(v);
      }
    }
  }

  /* 전체 대역 결과 저장 */
  for (int axis = 0; axis < 3; ++axis)
  {
    out->peak_ms2_x100[axis] = MS2_X100(max_val[axis]);
    out->rms_ms2_x100[axis] = MS2_X100(sqrtf(sum_sq[axis] / (float)total_parsed));
  }

  log_timing("broadband rms/peak", t_start, &t_prev);

  /* 10-1000Hz 대역 제한 통계 (FFT 기반) */
  for (int axis = 0; axis < 3; ++axis)
  {
    const int16_t *src = (axis == 0) ? g_ax : (axis == 1) ? g_ay : g_az;
    RC(compute_psd_acc_vel_axis(src, total_parsed, sensitivity_g_per_lsb,
                                g_cal_offset_lsb[axis],
                                &out->bl_rms_ms2_x100[axis], &out->bl_peak_ms2_x100[axis],
                                &out->bl_rms_mmps_x100[axis], &out->bl_peak_mmps_x100[axis]));

    log_timing(axis == 0 ? "Band axis X total" :
               (axis == 1 ? "Band axis Y total" : "Band axis Z total"),
               t_start, &t_prev);
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
  g_cal_offset_lsb[0] = 0.0f;
  g_cal_offset_lsb[1] = 0.0f;
  g_cal_offset_lsb[2] = 0.0f;

  lsm6dso_stats_t st = {0};
  int rc = lsm6dso_capture_once(&st, scale);
  if (rc)
  {
    return rc;
  }

  if (st.n == 0)
  {
    return -EIO;
  }

  float sx = 0.0f;
  float sy = 0.0f;
  float sz = 0.0f;

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

  LOG_INF("LSM6DSO calibration set: offset_lsb=(%.2f, %.2f, %.2f), n=%u, scale=%s",
          (double)g_cal_offset_lsb[0], (double)g_cal_offset_lsb[1],
          (double)g_cal_offset_lsb[2], st.n,
          (scale == LSM6DSO_SCALE_16G) ? "16g" : "4g");

  return 0;
}

void clear_calibration_lsm6dso(void)
{
  g_cal_offset_lsb[0] = 0.0f;
  g_cal_offset_lsb[1] = 0.0f;
  g_cal_offset_lsb[2] = 0.0f;
  LOG_INF("LSM6DSO calibration cleared");
}

int lsm6dso_dump_regs(const struct shell *shell)
{
  /* 기존 레지스터 덤프 인터페이스 유지 */
  uint8_t v = 0;

  (void)rd_u8(REG_WHO_AM_I, &v);
  shell_print(shell, "WHO: %02X", v);

  (void)rd_u8(REG_FIFO_CTRL1, &v);
  shell_print(shell, "FIFO1: %02X", v);

  (void)rd_u8(REG_FIFO_CTRL2, &v);
  shell_print(shell, "FIFO2: %02X", v);

  (void)rd_u8(REG_FIFO_CTRL3, &v);
  shell_print(shell, "FIFO3: %02X", v);

  (void)rd_u8(REG_FIFO_CTRL4, &v);
  shell_print(shell, "FIFO4: %02X", v);

  return 0;
}
