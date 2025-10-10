
#include "sensors.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

/* I2C controller */
static const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* SAADC */
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_buf;
static struct adc_channel_cfg ch_cfg = {
    .gain = ADC_GAIN_1_4,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 1, /* AIN1 = P0.03 */
};
static struct adc_sequence seq = {
    .channels = BIT(1),
    .buffer = &adc_buf,
    .buffer_size = sizeof(adc_buf),
    .resolution = 12,
};

/* Pressure sensor 0x28 (SSC) simple read */
int read_pressure_0x28(sensor_sample_t *out)
{
    if (!device_is_ready(i2c0)) return -ENODEV;
    uint8_t buf[4] = {0};
    int r = i2c_read(i2c0, buf, sizeof(buf), 0x28);
    if (r) return r;
    uint16_t bridge = ((buf[0] & 0x3F) << 8) | buf[1];
    const int OUTPUT_MIN = 1638, OUTPUT_MAX = 14745;
    const int P_MIN = -1020, P_MAX = 1020;
    int32_t p = (int32_t)(bridge - OUTPUT_MIN) * (P_MAX - P_MIN);
    p /= (OUTPUT_MAX - OUTPUT_MIN);
    p += P_MIN;
    out->p_value_x100 = p * 100;
    int16_t t_raw = ((buf[2] << 8) | (buf[3] & 0xE0)) >> 5;
    out->temperature_c_x100 = (int16_t)((t_raw * 977) / 10 - 5000); /* *100 */
    return 0;
}

int read_ntc_ain1_cx100(int16_t *cx100)
{
    if (!device_is_ready(adc_dev)) return -ENODEV;
    adc_channel_setup(adc_dev, &ch_cfg);
    int r = adc_read(adc_dev, &seq);
    if (r) return r;
    float adc_norm = (float)adc_buf / 4095.0f;
    float v = adc_norm * 0.6f;
    float temp_c = 25.0f + (v - 0.5f) * 100.0f;
    *cx100 = (int16_t)(temp_c * 100.0f);
    return 0;
}

int sensors_init(void){
    if (!device_is_ready(i2c0)) return -ENODEV;
    if (!device_is_ready(adc_dev)) return -ENODEV;
    return 0;
}
