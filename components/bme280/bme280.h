#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000
#define BME280_I2C_ADDRESS 0x77

void i2c_master_init(void);
void bme280_init(void);
void bme280_read_raw(int32_t *temp_raw, int32_t *press_raw, int32_t *hum_raw);
void bme280_calculate(int32_t temp_raw, int32_t press_raw, int32_t hum_raw,
                      float *temp_final, float *press_final, float *hum_final);

#endif // BME280_H
