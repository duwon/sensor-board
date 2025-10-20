#include "lsm6dso.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <math.h>
#include <string.h>

#define M_PI 3.14159

LOG_MODULE_REGISTER(lsm6dso, LOG_LEVEL_INF);

/* 간편 리턴 체크 매크로 */
#define RC(expr)           \
    do                     \
    {                      \
        int __rc = (expr); \
        if (__rc)          \
            return __rc;   \
    } while (0)

/* ==== Global capture buffers to avoid shell-thread stack overflow ==== */
#define FIFO_WTM_WORDS 999
#define FIFO_WORD_BYTES 7
#define FIFO_BURST_BYTES (FIFO_WTM_WORDS * FIFO_WORD_BYTES) // <-- 여기서만 1회 정의

/* Capture destination arrays (BSS) */
static int16_t g_ax[FIFO_WTM_WORDS];
static int16_t g_ay[FIFO_WTM_WORDS];
static int16_t g_az[FIFO_WTM_WORDS];
static uint8_t g_chunk[224]; /* 224 = 32 words * 7 bytes */

/* ====== 하드웨어/I2C 정의 ====== */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#define LSM6DSO_I2C_ADDR 0x6A

/* 레지스터 */
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

#define WHOAMI_EXPECTED 0x6C

/* CTRL3_C */
#define CTRL3_C_BDU BIT(6)
#define CTRL3_C_IF_INC BIT(2)

/* CTRL9_XL */
#define CTRL9_XL_I3C_DISABLE BIT(1)

/* CTRL1_XL: ODR=3.33kHz, FS */
#define ODR_XL_3k33 (0x0A << 4)
#define FS_XL_4G (0x2 << 2)
#define FS_XL_16G (0x1 << 2)

/* CTRL2_G: 파워다운 */
#define ODR_G_POWER_DOWN (0x0 << 4)

/* FIFO */
/* FIFO ODR = 3.33 kHz (ODR_FIFO[3:0] = 0x0A) */
#define ODR_FIFO_3k33 0x0A
#define BDR_XL_3k33 (0x0A)
#define FIFO_CTRL4_STOP_ON_WTM BIT(5)
#define FIFO_MODE_BYPASS 0x0
#define FIFO_MODE_FIFO 0x1

/* 샘플링 관련(사양서: 3.33 kHz) */
#define IMU_FS_HZ 3330.0f /* 3.33 kHz 근사 */

#define REG_STATUS_REG            0x1E   /* XLDA=bit0 */
#define XLDA_BIT                  0x01

/* ====== 내부 I2C 유틸 ====== */
static int wr_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_write(i2c0, buf, sizeof(buf), LSM6DSO_I2C_ADDR);
}

static int rd_u8(uint8_t reg, uint8_t *val)
{
    return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, val, 1);
}

static int fifo_diff_words(uint16_t *out)
{
    uint8_t st[2];
    int rc = i2c_burst_read(i2c0, LSM6DSO_I2C_ADDR, REG_FIFO_STATUS1, st, sizeof(st));
    if (rc)
        return rc;
    *out = ((uint16_t)(st[1] & 0x0F) << 8) | st[0];
    return 0;
}

static int rd_block(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, buf, len);
}

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

/* ====== 스케일: LSB→m/s^2 ====== */
float lsm6dso_scale_ms2_per_lsb(lsm6dso_fs_t fs)
{
    return (fs == LSM6DSO_FS_16G) ? 0.004784f : 0.001196f; /* ±16g / ±4g */
}

/* 새로 추가 */
#define REG_FIFO_CTRL5 0x0B
#define ODR_FIFO_3k33 0x0A /* FIFO clock = 3.33 kHz */

/* 선택: FIFO 모드 상수(가독성) */
#define FIFO_MODE_CONTINUOUS 0x6  /* Continuous mode */
#define FIFO_MODE_STOP_ON_WTM 0x1 /* 기존 값 */

/* ====== 초기화 ====== */
int lsm6dso_init(lsm6dso_fs_t fs)
{

    /* 0) 소프트 리셋(선택) */
    /* wr_u8(REG_CTRL3_C, CTRL3_C_SW_RESET); k_sleep(K_MSEC(2)); */

    RC(wr_u8(REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC)); // BDU=1, IF_INC=1
    RC(wr_u8(REG_CTRL9_XL, CTRL9_XL_I3C_DISABLE));        // I3C disable
    RC(wr_u8(REG_CTRL2_G, ODR_G_POWER_DOWN));             // Gyro off

    /* 1) FIFO BYPASS로 들어가서 깨끗이 비움 */
    RC(wr_u8(REG_FIFO_CTRL4, 0x00)); // MODE=000 (BYPASS)
    k_sleep(K_MSEC(2));

    /* 2) XL ODR/FS 설정 (원래 의도대로 3.33 kHz, ±4g 권장) */
    RC(wr_u8(REG_CTRL1_XL, (0x9 << 4) | FS_XL_4G)); // ODR_XL=0x9(3.33k), FS=±4g  → 기대값 0x98

    /* 3) FIFO 배치 속도(BDR) 설정 — XL만 3.333 kHz로 활성화, Gyro는 0 */
    RC(wr_u8(REG_FIFO_CTRL3, (0x0 << 4) | 0x9)); // BDR_GY=0000, BDR_XL=1001(3.333 kHz)

    /* 4) 워터마크(선택: 999 워드)와 STOP_ON_WTM 여부 */
    RC(wr_u8(REG_FIFO_CTRL1, (FIFO_WTM_WORDS & 0xFF)));
    RC(wr_u8(REG_FIFO_CTRL2, ((FIFO_WTM_WORDS >> 8) & 0x0F) /* | STOP_ON_WTM?0x80:0 */));

    /* 5) FIFO 모드 진입 (연속모드로 먼저 확인) */
    RC(wr_u8(REG_FIFO_CTRL4, 0x06)); // MODE=110 Continuous

    return 0;
}

/* FIFO BYPASS→FIFO로 빠른 리셋 */
static int fifo_reset_keep_stop_on_wtm(void)
{
    int rc = wr_u8(REG_FIFO_CTRL4, FIFO_CTRL4_STOP_ON_WTM | FIFO_MODE_BYPASS);
    if (rc)
        return rc;
    k_sleep(K_MSEC(2));
    rc = wr_u8(REG_FIFO_CTRL4, FIFO_CTRL4_STOP_ON_WTM | FIFO_MODE_FIFO);
    return rc;
}

/* 태그 체크(가속도) */
static bool is_accel_tag(uint8_t tag)
{
    return (tag & 0x1F) == 0x01;
}

/* ====== FFT 내장 구현(1024-point, 단정도, in-place) ======
 * - 실신호 입력: re[0..N-1], im[]는 0으로 시작
 * - inverse!=0 이면 역변환(스펙트럼 공액 후 동일 커널 사용)
 * - 정규화: inverse 수행 후 1/N 로 나눔
 */
#define NFFT 1024
static float tw_re[NFFT / 2], tw_im[NFFT / 2]; /* twiddle cache */
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
    const int log2n = 10; /* 2^10 = 1024 */

    /* bit reversal */
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

    /* stages */
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

/* Hann 창 생성 */
static void make_hann(float *w, int n)
{
    if (n <= 1)
    {
        w[0] = 1.f;
        return;
    }
    for (int i = 0; i < n; ++i)
    {
        w[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
    }
}

/* 주파수 대역(Hz)→bin 인덱스 범위 계산 */
static void band_to_bins(float fs, int nfft, float f_lo, float f_hi, int *kmin, int *kmax)
{
    int a = (int)ceilf(f_lo * nfft / fs);
    int b = (int)floorf(f_hi * nfft / fs);
    if (a < 1)
        a = 1; /* DC 제외 */
    if (b > nfft / 2)
        b = nfft / 2;
    if (b < a)
        b = a;
    *kmin = a;
    *kmax = b;
}

/* 한 축에 대해: 원시(lsb) → m/s^2 → Hann 창 → 1024로 제로패딩 → FFT → 대역외 0 → iFFT
 * → 시간파형에서 RMS/Peak 계산. (창/스케일 보정 대신 "대역제한된 시간신호"의 직접 RMS/Peak 사용)
 */
static void bandlimited_rms_peak_ms2_x100(
    const int16_t *lsb, uint16_t n, float lsb_to_ms2,
    float f_lo, float f_hi, int16_t *out_rms_x100, int16_t *out_peak_x100)
{
    static float re[NFFT], im[NFFT], win[NFFT];
    static bool  win_ready = false;
    if (!win_ready) { make_hann(win, NFFT); win_ready = true; }

    const int useN = (n < NFFT) ? n : NFFT;

    /* 0) DC(평균) 제거: LSB 평균을 빼고 물리단위로 변환 */
    float mean_lsb = 0.f;
    for (int i=0; i<useN; ++i) mean_lsb += lsb[i];
    mean_lsb = (useN > 0) ? (mean_lsb / useN) : 0.f;

    for (int i=0; i<useN; ++i) {
        float v = ( (float)lsb[i] - mean_lsb ) * lsb_to_ms2;  // 평균 제거
        float w = win[i];
        re[i] = v * w;  /* 창을 평균 제거 후에 곱함 */
        im[i] = 0.f;
    }
    for (int i=useN; i<NFFT; ++i) { re[i]=0.f; im[i]=0.f; }

    fft_radix2(re, im, 0);

    int kmin=0, kmax=0;
    band_to_bins(IMU_FS_HZ, NFFT, f_lo, f_hi, &kmin, &kmax);

    for (int k=1; k<NFFT/2; ++k) {
        if (k < kmin || k > kmax) {
            re[k]=im[k]=0.f;
            re[NFFT-k]=im[NFFT-k]=0.f;
        }
    }
    re[0]=im[0]=0.f; re[NFFT/2]=im[NFFT/2]=0.f;

    fft_radix2(re, im, 1);

    /* 시간영역에서 RMS/Peak (원래 샘플 길이만 고려) */
    float peak=0.f, sumsq=0.f; const int M=useN;
    for (int i=0; i<M; ++i) {
        float a = re[i];
        float au = a>0 ? a : -a;
        if (au > peak) peak = au;
        sumsq += a*a;
    }
    float rms = (M>0) ? sqrtf(sumsq / (float)M) : 0.f;

    *out_peak_x100 = (int16_t)(peak * 100.0f + 0.5f);
    *out_rms_x100  = (int16_t)(rms  * 100.0f + 0.5f);
}


/* ====== 캡처 & 통계 산출 ====== */
int lsm6dso_capture_once(lsm6dso_stats_t *out)
{
    if (!out) return -EINVAL;
    memset(out, 0, sizeof(*out));

    int rc;

    /* A) FIFO 완전 비활성 (바이패스) — 혼선 방지 */
    rc = wr_u8(REG_FIFO_CTRL4, 0x00);                   /* BYPASS */
    if (rc) return rc;
    k_sleep(K_MSEC(2));

    /* B) 0.3 s 동안 DRDY 폴링하며 직접 읽기
          - ODR=3.33 kHz 이므로 최대 ≈ 1100 샘플 가능
          - 안전상 최대 1024개까지만 저장 */
    const uint32_t window_ms = 330;
    const uint16_t N_MAX = 1024;

    uint32_t t0 = k_uptime_get_32();
    uint16_t n_accel = 0;

    while ((k_uptime_get_32() - t0) < window_ms) {
        uint8_t sr = 0;
        rc = rd_u8(REG_STATUS_REG, &sr);
        if (rc) return rc;

        if (sr & XLDA_BIT) {
            /* OUTX_L_A .. OUTZ_H_A (연속 6바이트) */
            uint8_t reg = REG_OUTX_L_A;
            uint8_t raw[6];
            rc = i2c_write_read(i2c0, LSM6DSO_I2C_ADDR, &reg, 1, raw, sizeof(raw));
            if (rc) return rc;

            int16_t x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
            int16_t y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
            int16_t z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

            if (n_accel < FIFO_WTM_WORDS && n_accel < N_MAX) {
                g_ax[n_accel] = x; g_ay[n_accel] = y; g_az[n_accel] = z;
                n_accel++;
            }
        } else {
            /* 수 μs 대기 */
            k_busy_wait(80); /* ≈ ODR 주기의 1/40 */
        }

        if (n_accel >= FIFO_WTM_WORDS) break;  /* 999개 모이면 */
    }

    out->n = n_accel;
    out->wtm_reached = (n_accel >= FIFO_WTM_WORDS);

    /* C) 통계 계산 (전체대역) */
    float s = lsm6dso_scale_ms2_per_lsb(LSM6DSO_FS_4G);
    float sumsq_x=0, sumsq_y=0, sumsq_z=0, peak_x=0, peak_y=0, peak_z=0;

    for (uint16_t i=0; i<n_accel; ++i) {
        float fx=g_ax[i]*s, fy=g_ay[i]*s, fz=g_az[i]*s;
        float axu=fabsf(fx), ayu=fabsf(fy), azu=fabsf(fz);
        if (axu>peak_x) peak_x=axu;
        if (ayu>peak_y) peak_y=ayu;
        if (azu>peak_z) peak_z=azu;
        sumsq_x+=fx*fx; sumsq_y+=fy*fy; sumsq_z+=fz*fz;
    }
    float rms_x = (n_accel>0)? sqrtf(sumsq_x/n_accel) : 0.f;
    float rms_y = (n_accel>0)? sqrtf(sumsq_y/n_accel) : 0.f;
    float rms_z = (n_accel>0)? sqrtf(sumsq_z/n_accel) : 0.f;

    out->peak_ms2_x100[0] = (int16_t)(peak_x * 100.f + 0.5f);
    out->peak_ms2_x100[1] = (int16_t)(peak_y * 100.f + 0.5f);
    out->peak_ms2_x100[2] = (int16_t)(peak_z * 100.f + 0.5f);
    out->rms_ms2_x100[0]  = (int16_t)(rms_x  * 100.f + 0.5f);
    out->rms_ms2_x100[1]  = (int16_t)(rms_y  * 100.f + 0.5f);
    out->rms_ms2_x100[2]  = (int16_t)(rms_z  * 100.f + 0.5f);

    /* D) 10–1000 Hz 대역 제한 */
    const float FLO = 10.0f, FHI = 1000.0f;
    bandlimited_rms_peak_ms2_x100(g_ax, n_accel, s, FLO, FHI,
                                  &out->bl_rms_ms2_x100[0], &out->bl_peak_ms2_x100[0]);
    bandlimited_rms_peak_ms2_x100(g_ay, n_accel, s, FLO, FHI,
                                  &out->bl_rms_ms2_x100[1], &out->bl_peak_ms2_x100[1]);
    bandlimited_rms_peak_ms2_x100(g_az, n_accel, s, FLO, FHI,
                                  &out->bl_rms_ms2_x100[2], &out->bl_peak_ms2_x100[2]);

    rd_u8(REG_WHO_AM_I, &out->whoami);

    return (out->n > 0) ? 0 : -EIO;
}
