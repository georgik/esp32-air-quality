#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    8        // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO    10        // gpio number for I2C master data
#define I2C_MASTER_NUM       I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ   100000    // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0    // I2C master doesn't use buffer
#define I2C_MASTER_RX_BUF_DISABLE 0    // I2C master doesn't use buffer

#define SHTC3_SENSOR_ADDR    0x70      // SHTC3 I2C address
#define SHTC3_CMD_MEASURE_H_T 0x7CA2   // Command for measuring humidity and temperature

static const char *TAG = "shtc3";


/**
 * Initialize the I2C master interface
 */
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
}

/**
 * Read temperature and humidity from the SHTC3 sensor
 */
static esp_err_t read_temperature_humidity(float *temperature, float *humidity) {
    uint8_t data[6];
    esp_err_t err;

    uint8_t command[] = {SHTC3_CMD_MEASURE_H_T >> 8, SHTC3_CMD_MEASURE_H_T & 0xFF};
    err = i2c_master_write_to_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, command, sizeof(command), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sending measure command failed");
        return err;
    }

    // Wait for the measurement to complete
    vTaskDelay(pdMS_TO_TICKS(20));

    // Read the measurement
    err = i2c_master_read_from_device(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reading data failed");
        return err;
    }

    // Convert the data
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];

    // Conversion formula from SHTC3 datasheet
    *temperature = -45 + 175 * (temp_raw / (float)0xFFFF) + (-3.61);
    *humidity = 100 * (hum_raw / (float)0xFFFF) + 2.47;

    return ESP_OK;
}

void app_main(void) {
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }

    float temperature = 0.0f, humidity = 0.0f;

    while (true) {
        ret = read_temperature_humidity(&temperature, &humidity);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", temperature, humidity);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature and humidity");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
    }
}
