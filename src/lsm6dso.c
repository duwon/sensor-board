/**
 * @file lsm6dso.c
 * @brief LSM6DSO IMU driver with 1024-sample FIFO capture and ISO 10816 style
 * band-limited vibration metrics.
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

/* FIFO capture: 512 FIFO words per burst, captured twice for 1024 samples. */
#define FIFO_WTM_WORDS 512
#define FIFO_BYTES_PER_WORD 7
#define FIFO_CAPTURE_CHUNKS 2
#define FIFO_TOTAL_WORDS (FIFO_WTM_WORDS * FIFO_CAPTURE_CHUNKS)

/* DSP constants from the PDF. */
#define FFT_SIZE 1024
#define SAMPLE_RATE_HZ 3333.0f
#define BAND_BIN_START 3
#define BAND_BIN_END 307
#define HANN_AMPLITUDE_CORRECTION 1.633f
#define EQ_PEAK_FACTOR 1.414213f
#define G_CONST_MS2 9.80665f
#define SENSITIVITY_4G_G_PER_LSB 0.000122f
#define SENSITIVITY_16G_G_PER_LSB 0.000488f

/* I2C and register definitions. */
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

#define CTRL3_C_BDU BIT(6)
#define CTRL3_C_IF_INC BIT(2)
#define CTRL9_XL_I3C_DISABLE BIT(1)

#define ODR_FIFO_3k33_SH ((uint8_t)(0x09 << 3))
#define CTRL1_ODR_3k33 0xA0
#define FS_XL_4G (0x2 << 2)
#define FS_XL_16G (0x1 << 2)

#define FIFO_MODE_BYPASS 0x0
#define FIFO_MODE_CONTINUOUS 0x6

#define MMPS_X100(v_mps) ((int16_t)((v_mps * 100000.0f) + ((v_mps >= 0.f) ? 0.5f : -0.5f)))
#define MS2_X100(v_ms2) ((int16_t)((v_ms2 * 100.0f) + ((v_ms2 >= 0.f) ? 0.5f : -0.5f)))

BUILD_ASSERT(FIFO_TOTAL_WORDS == FFT_SIZE, "FIFO capture must match FFT size");

/* FIFO sample storage. */
static int16_t g_ax[FIFO_TOTAL_WORDS];
static int16_t g_ay[FIFO_TOTAL_WORDS];
static int16_t g_az[FIFO_TOTAL_WORDS];
static uint8_t g_fifo_raw[FIFO_WTM_WORDS * FIFO_BYTES_PER_WORD];

/* CMSIS-DSP working state. */
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

static inline uint32_t now_ms(void)
{
  return k_uptime_get_32();
}

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

static int lsm6dso_set_fs(lsm6dso_scale_t scale)
{
  const bool use_16g = (scale == LSM6DSO_SCALE_16G);
  const uint8_t fs_bits = use_16g ? FS_XL_16G : FS_XL_4G;

  return wr_u8(REG_CTRL1_XL, (uint8_t)(CTRL1_ODR_3k33 | fs_bits));
}

static int lsm6dso_apply_fifo_base(void)
{
  RC(wr_u8(REG_CTRL10_C, 0x00));
  RC(wr_u8(REG_FIFO_CTRL3, 0x09));
  RC(wr_u8(REG_FIFO_CTRL1, (uint8_t)(FIFO_WTM_WORDS & 0xFF)));
  RC(wr_u8(REG_FIFO_CTRL2, (uint8_t)((FIFO_WTM_WORDS >> 8) & 0x0F)));
  RC(wr_u8(REG_FIFO_CTRL4, 0x00));

  return 0;
}

static int fifo_set_mode(uint8_t mode)
{
  int rc = wr_u8(REG_FIFO_CTRL5, ODR_FIFO_3k33_SH);
  if (rc)
  {
    return rc;
  }

  return wr_u8(REG_FIFO_CTRL4, (uint8_t)(mode & 0x07));
}

static int lsm6dso_prepare_rfft(void)
{
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
  int rc = lsm6dso_prepare_rfft();
  if (rc < 0)
  {
    return rc;
  }

  lsm6dso_prepare_hann_window();
  return 0;
}

int lsm6dso_init(void)
{
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

  float mean_raw = 0.0f;
  for (uint16_t i = 0; i < n; ++i)
  {
    mean_raw += ((float)lsb[i] - offset_lsb);
  }
  mean_raw /= (float)n;

  for (uint16_t i = 0; i < n; ++i)
  {
    float raw_zeroed = ((float)lsb[i] - offset_lsb) - mean_raw;
    float accel_g = raw_zeroed * sensitivity_g_per_lsb;
    float accel_ms2 = accel_g * G_CONST_MS2;
    g_fft_buffer[i] = accel_ms2 * g_hann_window[i];
  }

  log_timing("Band axis: mean/window", t_start, &t_prev);

  arm_rfft_fast_f32(&g_rfft_inst, g_fft_buffer, g_fft_output, 0);
  log_timing("Band axis: RFFT", t_start, &t_prev);

  arm_cmplx_mag_f32(g_fft_output, g_mag_buffer, FFT_SIZE / 2);
  log_timing("Band axis: magnitude", t_start, &t_prev);

  const float df = SAMPLE_RATE_HZ / (float)FFT_SIZE;
  float sum_accel_sq = 0.0f;
  float sum_vel_sq = 0.0f;

  for (int k = BAND_BIN_START; k <= BAND_BIN_END; ++k)
  {
    float freq = (float)k * df;
    float ak = (g_mag_buffer[k] / (FFT_SIZE / 2.0f)) * HANN_AMPLITUDE_CORRECTION;

    if (g_calc_acc)
    {
      sum_accel_sq += ak * ak;
    }

    if (g_calc_vel)
    {
      float vk = (ak / (2.0f * (float)M_PI * freq)) * 1000.0f;
      sum_vel_sq += vk * vk;
    }
  }

  log_timing("Band axis: band sum", t_start, &t_prev);

  if (g_calc_acc)
  {
    float acc_rms = sqrtf(sum_accel_sq / 2.0f);
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
    float vel_rms = sqrtf(sum_vel_sq / 2.0f);
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

  uint16_t total_parsed = 0;

  for (int chunk = 0; chunk < FIFO_CAPTURE_CHUNKS; ++chunk)
  {
    uint32_t chunk_wait_start = now_ms();
    bool ready = false;

    while ((now_ms() - chunk_wait_start) < 250)
    {
      uint8_t st[2];
      if (i2c_write_read(i2c0, LSM6DSO_I2C_ADDR,
                         (uint8_t[]){REG_FIFO_STATUS1}, 1, st, 2) != 0)
      {
        return -EIO;
      }

      uint16_t diff = ((uint16_t)(st[1] & 0x0F) << 8) | st[0];
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

    log_timing(chunk == 0 ? "chunk0 wait WTM" : "chunk1 wait WTM", t_start, &t_prev);

    uint8_t reg_addr = REG_FIFO_DATA_OUT_TAG;
    if (i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg_addr, 1,
                       g_fifo_raw, sizeof(g_fifo_raw)) != 0)
    {
      LOG_ERR("FIFO read fail at chunk %d", chunk);
      return -EIO;
    }

    log_timing(chunk == 0 ? "chunk0 burst read" : "chunk1 burst read", t_start, &t_prev);

    uint16_t base_idx = (uint16_t)(chunk * FIFO_WTM_WORDS);
    uint16_t acc_valid_count = 0;
    uint16_t non_acc_count = 0;

    for (size_t i = 0; i < sizeof(g_fifo_raw); i += FIFO_BYTES_PER_WORD)
    {
      uint8_t tag_val = (uint8_t)((g_fifo_raw[i] >> 3) & 0x1F);
      if (tag_val != 0x01 && tag_val != 0x02)
      {
        non_acc_count++;
        continue;
      }

      if (acc_valid_count >= FIFO_WTM_WORDS)
      {
        LOG_ERR("Chunk %d produced too many accel samples", chunk);
        return -EIO;
      }

      int16_t x = (int16_t)((uint16_t)g_fifo_raw[i + 1] |
                            ((uint16_t)g_fifo_raw[i + 2] << 8));
      int16_t y = (int16_t)((uint16_t)g_fifo_raw[i + 3] |
                            ((uint16_t)g_fifo_raw[i + 4] << 8));
      int16_t z = (int16_t)((uint16_t)g_fifo_raw[i + 5] |
                            ((uint16_t)g_fifo_raw[i + 6] << 8));

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
    log_timing(chunk == 0 ? "chunk0 parse" : "chunk1 parse", t_start, &t_prev);
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

  for (int axis = 0; axis < 3; ++axis)
  {
    out->peak_ms2_x100[axis] = MS2_X100(max_val[axis]);
    out->rms_ms2_x100[axis] = MS2_X100(sqrtf(sum_sq[axis] / (float)total_parsed));
  }

  log_timing("broadband rms/peak", t_start, &t_prev);

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
