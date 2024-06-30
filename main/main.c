#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "drone.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <string.h>
#include "esp_err.h"

#include "freertos/task.h"





static int s_retry_num = 0;





static const char *TAG = "i2c-simple-example";

// PID variables
float error_roll, error_pitch;    // Error terms for roll and pitch
float integral_roll, integral_pitch;  // Integral terms for roll and pitch
float derivative_roll, derivative_pitch;  // Derivative terms for roll and pitch
float pid_output_roll, pid_output_pitch;  // PID outputs for roll and pitch
float last_error_roll, last_error_pitch;  // Last error terms for roll and pitc


float accel_x, accel_y, accel_z;  // Accelerometer data
float gyro_x, gyro_y, gyro_z;     // Gyroscope data

void read_mpu6050_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void calculate_pid(void);


void pid_init(void) {
    integral_roll = 0.0;
    integral_pitch = 0.0;
    last_error_roll = 0.0;
    last_error_pitch = 0.0;
}






// PID calculation function
void calculate_pid(void) {
    // Calculate errors

    read_mpu6050_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    error_roll = (TARGET_ROLL - accel_x) * ACCEL_SENSITIVITY;
    error_pitch = (TARGET_PITCH - accel_y) * ACCEL_SENSITIVITY;

    // Calculate integral terms
    integral_roll += error_roll * PID_SAMPLE_TIME_MS / 1000.0;
    integral_pitch += error_pitch * PID_SAMPLE_TIME_MS / 1000.0;

    // Calculate derivative terms
    derivative_roll = (error_roll - last_error_roll) / (PID_SAMPLE_TIME_MS / 1000.0);
    derivative_pitch = (error_pitch - last_error_pitch) / (PID_SAMPLE_TIME_MS / 1000.0);

    // Calculate PID outputs
    pid_output_roll = KP * error_roll + KI * integral_roll + KD * derivative_roll;
    pid_output_pitch = KP * error_pitch + KI * integral_pitch + KD * derivative_pitch;

    // Apply PID output limits
    if (pid_output_roll > PID_OUTPUT_MAX) {
        pid_output_roll = PID_OUTPUT_MAX;
    } else if (pid_output_roll < PID_OUTPUT_MIN) {
        pid_output_roll = PID_OUTPUT_MIN;
    }

    if (pid_output_pitch > PID_OUTPUT_MAX) {
        pid_output_pitch = PID_OUTPUT_MAX;
    } else if (pid_output_pitch < PID_OUTPUT_MIN) {
        pid_output_pitch = PID_OUTPUT_MIN;
    }

    // Update last error terms
    last_error_roll = error_roll;
    last_error_pitch = error_pitch;

    // Print PID outputs (for debugging)
    ESP_LOGI(TAG, "PID Output - Roll: %.2f, Pitch: %.2f", pid_output_roll, pid_output_pitch);

    // TODO: Apply PID outputs to control your drone's motors or servos
}


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


    *ax = read_raw_data(MPU6050_ACCEL_XOUT_H_REG_ADDR) / ACCEL_SENSITIVITY;
    *ay = read_raw_data(MPU6050_ACCEL_YOUT_H_REG_ADDR) / ACCEL_SENSITIVITY;
    *az = read_raw_data(MPU6050_ACCEL_ZOUT_H_REG_ADDR) / ACCEL_SENSITIVITY;

    *gx = read_raw_data(MPU6050_GYRO_XOUT_H_REG_ADDR) / GYRO_SENSITIVITY;
    *gy = read_raw_data(MPU6050_GYRO_YOUT_H_REG_ADDR) / GYRO_SENSITIVITY;
    *gz = read_raw_data(MPU6050_GYRO_ZOUT_H_REG_ADDR) / GYRO_SENSITIVITY;
}

/**

 * @brief i2c master initialization

 */

static esp_err_t i2c_master_init(void)
{    int i2c_master_port = I2C_MASTER_NUM;

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

static void motor_pwm_init(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // 10-bit PWM upplösning
        .freq_hz = MOTOR_PWM_FREQ,            // PWM frekvens
        .speed_mode = LEDC_LOW_SPEED_MODE,   // Hög hastighetsläge
        .timer_num = LEDC_TIMER_0             // Timer 0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = MOTOR1_PWM_GPIO,          // GPIO nummer för motor 1
        .speed_mode = LEDC_LOW_SPEED_MODE,   // Hög hastighetsläge
        .channel = MOTOR1_PWM_CHANNEL,        // PWM kanal för motor 1
        .intr_type = LEDC_INTR_DISABLE,       // Avaktivera avbrott
        .timer_sel = LEDC_TIMER_0,            // Använd Timer 0
        .duty = 0                             // Start duty cycle 0 (motor stannad)
    };
    ledc_channel_config(&ledc_channel1);

    
}


static void set_motor_speed(float pid_output_roll, float pid_output_pitch) {
    // Beräkna duty cycle (0-1023) baserat på PID-utdata
    uint32_t duty1 = (uint32_t)((pid_output_roll / PID_OUTPUT_MAX) * 1023);
    uint32_t duty2 = (uint32_t)((pid_output_pitch / PID_OUTPUT_MAX) * 1023);

    // Sätt PWM duty cycle för motorerna
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR1_PWM_CHANNEL, duty1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR1_PWM_CHANNEL);

    
}



void app_main(void)
{
     // Initialize PID controller
    pid_init();

    // Initialisera PWM för motorer
    motor_pwm_init();

   

    

    uint8_t data[2];    
    
    
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

// Perform PID calculations
    calculate_pid();

    float ax, ay, az;  // Accelerometerdata
    float gx, gy, gz;  // Gyroskopdata

// Anropa funktionen för att läsa sensorvärden
    read_mpu6050_data(&ax, &ay, &az, &gx, &gy, &gz);

    // Initialisera PWM för motorer
    motor_pwm_init();

// Nu är variablerna ax, ay, az, gx, gy, gz fyllda med aktuella sensorvärden i enheter (g och °/s)
// Du kan nu använda dessa värden för att göra vidare beräkningar eller logga dem, t.ex.:
    ESP_LOGI(TAG, "Acc: X=%.2f g, Y=%.2f g, Z=%.2f g", ax, ay, az);
    ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", gx, gy, gz);


    vTaskDelay(pdMS_TO_TICKS(1000));   


}


    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
 
  

  
}

