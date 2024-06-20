#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "drone.h"




static const char *TAG = "i2c-simple-example";






/**

 * @brief Read a sequence of bytes from a MPU9250 sensor registers

 */

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len){

    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

}
/**

 * @brief Write a byte to a MPU9250 sensor register

 */

static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{   int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;

}


int16_t read_raw_data(int addr){
    int16_t high_byte,low_byte,value;
    uint8_t data[2];

    ESP_ERROR_CHECK(mpu6050_register_read(addr ,data, 1));
    //ESP_LOGI(TAG, "X_H = %X", data[0]);

    high_byte = data[0]<<8;
   // vTaskDelay(pdMS_TO_TICKS(500));
    

    ESP_ERROR_CHECK(mpu6050_register_read(addr+1, data+1, 1));
    //ESP_LOGI(TAG, "X_L = %X", data[0]);
    low_byte = data[1];
   // ESP_LOGI(TAG, "Acc X = %u", accXdata);
    //vTaskDelay(pdMS_TO_TICKS(50));
    value = (high_byte | low_byte);


    return value;
    }

    void read_mpu6050_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){


    *ax = read_raw_data(MPU6050_ACCEL_XOUT_H_REG_ADDR) / 16384.0;
    *ay = read_raw_data(MPU6050_ACCEL_YOUT_H_REG_ADDR) / 16384.0;
    *az = read_raw_data(MPU6050_ACCEL_ZOUT_H_REG_ADDR) / 16384.0;

    *gx = read_raw_data(MPU6050_GYRO_XOUT_H_REG_ADDR) / 131.0;
    *gy = read_raw_data(MPU6050_GYRO_YOUT_H_REG_ADDR) / 131.0;
    *gz = read_raw_data(MPU6050_GYRO_ZOUT_H_REG_ADDR) / 131.0;
}
    




/**

 * @brief i2c master initialization

 */

static esp_err_t i2c_master_init(void)

{

    int i2c_master_port = I2C_MASTER_NUM;




    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,

    };


    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

}

void app_main(void)

{
    uint8_t data[2];    
    uint16_t accXdataX;
    uint16_t accXdataY;
    uint16_t accXdataZ;
    
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    vTaskDelay(pdMS_TO_TICKS(500));




    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */

    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mpu6050_register_write_byte(0x6B, 0x00));
    vTaskDelay(pdMS_TO_TICKS(500));




    //ESP_LOGI(TAG, "WHO_AM_I2 = %X", data[1]);

    //ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mpu6050_register_write_byte(0x23, 1));
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mpu6050_register_read(0x23, data, 1<<3));
    ESP_LOGI(TAG, "Started = %X", data[0]);

vTaskDelay(pdMS_TO_TICKS(500));


while (1){
float ax, ay, az;  // Accelerometerdata
float gx, gy, gz;  // Gyroskopdata

// Anropa funktionen för att läsa sensorvärden
read_mpu6050_data(&ax, &ay, &az, &gx, &gy, &gz);

// Nu är variablerna ax, ay, az, gx, gy, gz fyllda med aktuella sensorvärden i enheter (g och °/s)
// Du kan nu använda dessa värden för att göra vidare beräkningar eller logga dem, t.ex.:
ESP_LOGI(TAG, "Acc: X=%.2f g, Y=%.2f g, Z=%.2f g", ax, ay, az);
ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", gx, gy, gz);


    vTaskDelay(pdMS_TO_TICKS(5000));   


}


    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
 
  

  
}

