#include "bme280.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Calibration parameters
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5, dig_H6;

static int32_t t_fine; // Shared variable for compensation calculations

static i2c_master_dev_handle_t i2c_dev_handle;
static const char *TAG = "BME280";

// I2C initialization
void i2c_master_init() {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev_handle));
    ESP_LOGI(TAG, "I2C master initialized successfully.");
}

// Read calibration data from BME280
static void read_calibration_data() {
    uint8_t calib[26];
    uint8_t reg = 0x88;
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_receive(i2c_dev_handle, calib, 26, pdMS_TO_TICKS(1000)));

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];

    reg = 0xA1;
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_receive(i2c_dev_handle, &dig_H1, 1, pdMS_TO_TICKS(1000)));

    reg = 0xE1;
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    uint8_t hum_calib[7];
    ESP_ERROR_CHECK(i2c_master_receive(i2c_dev_handle, hum_calib, 7, pdMS_TO_TICKS(1000)));

    dig_H2 = (hum_calib[1] << 8) | hum_calib[0];
    dig_H3 = hum_calib[2];
    dig_H4 = (hum_calib[3] << 4) | (hum_calib[4] & 0x0F);
    dig_H5 = (hum_calib[5] << 4) | (hum_calib[4] >> 4);
    dig_H6 = hum_calib[6];
    ESP_LOGI(TAG, "Calibration data read successfully.");
}

// Initialize BME280
void bme280_init() {
    uint8_t reset_command = 0xB6;
    uint8_t reset_register = 0xE0;

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &reset_register, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &reset_command, 1, pdMS_TO_TICKS(1000)));

    uint8_t ctrl_meas[] = {0xF4, 0x27};
    uint8_t config[] = {0xF5, 0xA0};
    uint8_t ctrl_hum[] = {0xF2, 0x01};

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, ctrl_hum, 2, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, ctrl_meas, 2, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, config, 2, pdMS_TO_TICKS(1000)));

    read_calibration_data();
    ESP_LOGI(TAG, "BME280 initialized successfully.");
}

// Read raw data
void bme280_read_raw(int32_t *temp_raw, int32_t *press_raw, int32_t *hum_raw) {
    uint8_t reg = 0xF7;
    uint8_t data[8];

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_receive(i2c_dev_handle, data, 8, pdMS_TO_TICKS(1000)));

    *press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    *temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    *hum_raw = (data[6] << 8) | data[7];
}

// Calculate compensated values
void bme280_calculate(int32_t temp_raw, int32_t press_raw, int32_t hum_raw,
                      float *temp_final, float *press_final, float *hum_final) {
    int32_t var1, var2, T;
    var1 = ((((temp_raw >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((temp_raw >> 4) - ((int32_t)dig_T1)) * ((temp_raw >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >>
           14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    *temp_final = T / 100.0f;

    int64_t p1, p2;
    p1 = ((int64_t)t_fine) - 128000;
    p2 = p1 * p1 * (int64_t)dig_P6;
    p2 = p2 + ((p1 * (int64_t)dig_P5) << 17);
    p2 = p2 + (((int64_t)dig_P4) << 35);
    p1 = ((p1 * p1 * (int64_t)dig_P3) >> 8) + ((p1 * (int64_t)dig_P2) << 12);
    p1 = (((((int64_t)1) << 47) + p1)) * ((int64_t)dig_P1) >> 33;
    if (p1 == 0) {
        *press_final = 0; // Avoid division by zero
    } else {
        int64_t p = 1048576 - press_raw;
        p = (((p << 31) - p2) * 3125) / p1;
        p1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + p1 + p2) >> 8) + (((int64_t)dig_P7) << 4);
        *press_final = p / 256.0f / 100.0f;
    }

    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((hum_raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)dig_H2) +
                   8192) >>
                  14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *hum_final = (v_x1_u32r >> 12) / 1024.0f;
}
