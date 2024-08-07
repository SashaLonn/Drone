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
#include "esp_system.h"
#include <esp_http_server.h>
#include "nvs_flash.h"
#include "connect_wifi.h"
#include "freertos/task.h"
#include "spi_flash_mmap.h"



// HTML page served by the web server
char html_page[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>ESP Web Server</title>"
  "<link rel=\"stylesheet\" href=\"https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.4/css/all.min.css\">"
  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
  
  "<style>"
    "body {"
      "font-family: Arial, Helvetica, sans-serif;"
      "text-align: center;"
      "margin: 0;"
    "}"
    ".topnav {"
     "overflow: hidden;"
      "background-color: #ff99cc;"
      "color: white;"
      "font-size: 1.2rem;"
      
    "}"
    ".content {"
      "padding: 20px;"
    "}"
    ".cards {"
      "display: flex;"
      "justify-content: center;"
      "flex-wrap: wrap;"
    "}"
    ".card {"
      "background: #f4f4f4;"
      "padding: 20px;"
      "margin: 20px;"
      "box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);"
      "width: 300px;"
      "border-radius: 5px;"
    "}"
    ".card-title {"
      "font-size: 1.5rem;"
      "margin-bottom: 10px;"
    "}"
    ".reading {"
      "font-size: 1.2rem;"
    "}"
    ".cube-content {"
      "margin-top: 20px;"
    "}"
    ".blue-icon {"
  "color: blue;" 
"}"
    "#3Dcube {"
      "width: 300px;"
      "height: 300px;"
      "margin: 0 auto;"
    "}"
  "</style>"
"</head>"
"<body>"
  "<div class=\"topnav\">"
    "<h1> MPU6050 <i class=\"fas fa-drone blue-icon\" aria-hidden=\"true\" style=\"font-size: 1em;\"></i></h1>"
  "</div>"
  "<div class=\"content\">"
    "<div class=\"cards\">"
      "<div class=\"card\">"
        "<p class=\"card-title\">GYROSCOPE</p>"
        "<p><span class=\"reading\">X: <span id=\"gyroX\">%2f</span> rad</span></p>"
        "<p><span class=\"reading\">Y: <span id=\"gyroY\">%2f</span> rad</span></p>"
        "<p><span class=\"reading\">Z: <span id=\"gyroZ\">%2f</span> rad</span></p>"
      "</div>"
      "<div class=\"card\">"
        "<p class=\"card-title\">ACCELEROMETER</p>"
        "<p><span class=\"reading\">X: <span id=\"accX\">%2f</span> ms<sup>2</sup></span></p>"
        "<p><span class=\"reading\">Y: <span id=\"accY\">%2f</span> ms<sup>2</sup></span></p>"
        "<p><span class=\"reading\">Z: <span id=\"accZ\">%2f</span> ms<sup>2</sup></span></p>"
      "</div>"
    "</div>"
   
    "<div class=\"cube-content\">"
      "<div id=\"3Dcube\"></div>"
    "</div>"
  "</div>"
  "<script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/107/three.min.js\"></script>"
  "<script>"
    "let scene, camera, renderer, cube;"
     "const threshold = 0.02;"
    "function init3D(){"
      "scene = new THREE.Scene();"
      "scene.background = new THREE.Color(0xffffff);"
      "camera = new THREE.PerspectiveCamera(75, 300 / 300, 0.1, 1000);"
      "renderer = new THREE.WebGLRenderer({ antialias: true });"
      "renderer.setSize(300, 300);"
      "document.getElementById('3Dcube').appendChild(renderer.domElement);"
      "const geometry = new THREE.BoxGeometry(4, 4, 4);"
      "const material = new THREE.MeshStandardMaterial({ color: 0x000000 });"
      "cube = new THREE.Mesh(geometry, material);"
      "scene.add(cube);"  
      "camera.position.x = 7;"
       "camera.lookAt(new THREE.Vector3(0, 0, 0));"
      "renderer.render(scene, camera);"
    "};"

     "function filterNoise(value) {"
        "return Math.abs(value) < threshold ? 0 : value;"
   " };"
    "function fetchData() {"
      "fetch('/sensor')"
        ".then(response => response.json())"
        ".then(data => {"
         "document.getElementById('gyroX').innerText = data.gyroX.toFixed(2);"
          "document.getElementById('gyroY').innerText = data.gyroY.toFixed(2);"
          "document.getElementById('gyroZ').innerText = data.gyroZ.toFixed(2);"
        
          "document.getElementyId('accX').innerText = data.accX.toFixed(2);"
          "document.getElementById('accY').innerText = data.accY.toFixed(2);"
          "document.getElementById('accZ').innerText = data.accZ.toFixed(2);"
          "cube.rotation.x = filterNoise(data.accX);"
          "cube.rotation.y = filterNoise(data.accY);"
          "cube.rotation.z = filterNoise(data.accZ);"
          
          "renderer.render(scene, camera);"
        "})"
        ".catch(error => console.error('Error:', error));"
    "};"
    "window.onload = () => {"
      "init3D();"
      "setInterval(fetchData, 10);"
    "};"
  "</script>"
   "</body>"
"</html>";


static const char *TAG = "WEb server";


float accel_x, accel_y, accel_z;  // Accelerometer data
float gyro_x, gyro_y, gyro_z;     // Gyroscope data

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

// Read 16-bit raw data from a MPU9250 sensor register
int16_t read_raw_data(int addr){
    int16_t high_byte,low_byte,value;
    uint8_t data[2];

    // Read high byte of data
    ESP_ERROR_CHECK(mpu6050_register_read(addr ,data, 1));    
    high_byte = data[0]<<8;
       
    // Read low byte of data
    ESP_ERROR_CHECK(mpu6050_register_read(addr+1, data+1, 1));
    low_byte = data[1];
   
    // Combine high and low bytes into a 16-bit value
    value = (high_byte | low_byte);
    return value;
    }


    void read_mpu6050_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){

      // Read accelerometer data and convert to real-world values
    *ax = read_raw_data(MPU6050_ACCEL_XOUT_H_REG_ADDR) / ACCEL_SENSITIVITY;
    *ay = read_raw_data(MPU6050_ACCEL_YOUT_H_REG_ADDR) / ACCEL_SENSITIVITY;
    *az = read_raw_data(MPU6050_ACCEL_ZOUT_H_REG_ADDR) / ACCEL_SENSITIVITY;
   // Read gyroscope data and convert to real-world values
    *gx = read_raw_data(MPU6050_GYRO_XOUT_H_REG_ADDR) / GYRO_SENSITIVITY;
    *gy = read_raw_data(MPU6050_GYRO_YOUT_H_REG_ADDR) / GYRO_SENSITIVITY;
    *gz = read_raw_data(MPU6050_GYRO_ZOUT_H_REG_ADDR) / GYRO_SENSITIVITY;
}

/**

 * @brief i2c master initialization

 */
static esp_err_t i2c_master_init(void)
{
 int i2c_master_port = I2C_MASTER_NUM;

    // Configure I2C parameters
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,        // Set I2C mode to master
        .sda_io_num = I2C_MASTER_SDA_IO, // Set SDA pin number
        .scl_io_num = I2C_MASTER_SCL_IO, // Set SCL pin number
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Enable SDA pull-up resistors
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Enable SCL pull-up resistors
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Set I2C clock speed
    };
     // Configure I2C with specified parameters
    i2c_param_config(i2c_master_port, &conf);

    // Install I2C driver
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void motor_gpio_init(void) {
    // Configuration structure for GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR1_PWM_GPIO) | (1ULL << MOTOR2_PWM_GPIO), // Bitmask of GPIO pins to configure
        .mode = GPIO_MODE_OUTPUT,              // Set GPIO mode to output
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up resistors
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistors
        .intr_type = GPIO_INTR_DISABLE         // Disable GPIO interrupt
    }; 

    // Configure GPIO pins based on the provided configuration
    gpio_config(&io_conf);
}


static void motor_pwm_init(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // 10-bit PWM upplösning
        .freq_hz = MOTOR_PWM_FREQ,            // PWM frekvens
        .speed_mode = LEDC_LOW_SPEED_MODE,   //  Low speed_mode
        .timer_num = LEDC_TIMER_0             // Timer 0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = MOTOR1_PWM_GPIO,          // GPIO nummer för motor 1
        .speed_mode = LEDC_LOW_SPEED_MODE,   // Low speed_mode
        .channel = MOTOR1_PWM_CHANNEL,        // PWM chanal for motor 1
        .intr_type = LEDC_INTR_DISABLE,       // Avaktivera avbrott
        .timer_sel = LEDC_TIMER_0,            // Use Timer 0
        .duty = 0                             // Start duty cycle 0 (motor stannad)
    };
    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = MOTOR2_PWM_GPIO,          // GPIO nummer för motor 2
        .speed_mode = LEDC_LOW_SPEED_MODE,    //  Low speed_mode
        .channel = MOTOR2_PWM_CHANNEL,        //  PWM chanal for motor 2
        .intr_type = LEDC_INTR_DISABLE,       // Avaktivera avbrott
        .timer_sel = LEDC_TIMER_0,            // Use Timer 0
        .duty = 0                             // Start duty cycle 0 (motor stannad)
    };
    ledc_channel_config(&ledc_channel2);

    
}


void motor_control_task(void *pvParameters) {
    
    while (1) {
        // Read accelerometer data
        read_mpu6050_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

        // Check conditions to adjust motor speed
        if (accel_x < -0.5) {
            // Calculate new duty cycle based on accelerometer data
            uint32_t duty = 1023; // Example: 0 to 1000 based on the ay value (adjust for your hardware)
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ESP_LOGI(TAG, "Motor duty set to %u", (unsigned int)duty);
               
        } 
        else if (accel_x > 0.5) {
            // Calculate new duty cycle based on accelerometer data
            uint32_t duty = 1023; // Example: 0 to 1000 based on the ay value (adjust for your hardware)
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ESP_LOGI(TAG, "Motor 2 duty set to %u", (unsigned int)duty);
               
        } else if (accel_y > 0.4) {
            // Calculate new duty cycle based on accelerometer data
            uint32_t duty = 1023; // Example: 0 to 1000 based on the ay value (adjust for your hardware)
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ESP_LOGI(TAG, "2 Motors duty set to %u", (unsigned int)duty);
               
        } else {
            // Stop the motors if no condition is met
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t send_web_page(httpd_req_t *req)
{
    int response; // Variable to store the response status

    // Read sensor data
    read_mpu6050_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

    // Prepare response data with formatted HTML page including sensor data
    char response_data[sizeof(html_page) + 50]; // Allocate space for the response data
    memset(response_data, 0, sizeof(response_data)); // Clear the response_data buffer
    sprintf(response_data, html_page, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z); // Format HTML page with sensor data

    // Send HTTP response
    response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);

    return response; // Return the response status (ESP_OK or error code)
}


esp_err_t get_req_handler(httpd_req_t *req)
{
    return send_web_page(req);
}

esp_err_t get_sensor_data(httpd_req_t *req)
{  
      int response;
    char json_response[1000]; // Justera storleken beroende på JSON-datans komplexitet
    memset( json_response, 0, sizeof( json_response));

    // Skapa JSON-strängen med sensorvärden
    sprintf(json_response, "{\"accX\": %.2f, \"accY\": %.2f, \"accZ\": %.2f, \"gyroX\": %.2f, \"gyroY\": %.2f, \"gyroZ\": %.2f}",
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

    // Sätt HTTP-headers för JSON
     response = httpd_resp_send(req, json_response,HTTPD_RESP_USE_STRLEN);

    // Skicka JSON-svaret till klienten
   

    return response;
}

httpd_uri_t uri_get = {
    .uri = "/",                // URI path for this handler
    .method = HTTP_GET,        // HTTP method (GET in this case)
    .handler = get_req_handler, // Function pointer to handle GET requests
    .user_ctx = NULL           // User context (can be used to pass custom data to the handler)
};

httpd_uri_t uri_get_sensor = {
    .uri = "/sensor",          // URI path for this handler
    .method = HTTP_GET,        // HTTP method (GET in this case)
    .handler = get_sensor_data, // Function pointer to handle GET requests for sensor data
    .user_ctx = NULL           // User context (can be used to pass custom data to the handler)
};



httpd_handle_t setup_server(void) {
    // Configure default HTTP server settings
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Adjust server configuration settings
    config.stack_size = 6000;  // Increase stack size if necessary
  

    httpd_handle_t server = NULL;

    // Start the HTTP server with the specified configuration
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers for specific endpoints
        httpd_register_uri_handler(server, &uri_get);         // Register URI handler for GET requests
        httpd_register_uri_handler(server, &uri_get_sensor);  // Register URI handler for sensor data endpoint
    }

    return server;  // Return handle to the HTTP server
}



void app_main(void) {
    // Initialize GPIO pins for motor control
    motor_gpio_init();

    // Initialize PWM for motor control
    motor_pwm_init();

    // Initialize NVS (Non-Volatile Storage) flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // If NVS initialization fails due to no free pages or new version found, erase and reinitialize
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);  // Check if NVS initialization was successful

    uint8_t data[2]; // Buffer for I2C data

    // Initialize I2C master
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Read WHO_AM_I register from MPU6050
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure MPU6050
    ESP_ERROR_CHECK(mpu6050_register_write_byte(0x6B, 0x00)); // Power management register
    vTaskDelay(pdMS_TO_TICKS(500));

    // Connect to Wi-Fi
    connect_wifi();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure MPU6050 to start
    ESP_ERROR_CHECK(mpu6050_register_write_byte(0x23, 1)); // Example register address and value
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mpu6050_register_read(0x23, data, 1)); // Read back the value for verification
    ESP_LOGI(TAG, "Started = %X", data[0]);

    // Create a task for motor control
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);

    // If Wi-Fi connection is successful, setup and run a web server
    if (wifi_connect_status) {
        setup_server();
        ESP_LOGI(TAG, "Web Server is up and running\n");

        // Continuously read and log MPU6050 data
        while (wifi_connect_status) {
            read_mpu6050_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
            ESP_LOGI(TAG, "Acc: X=%.2f g, Y=%.2f g, Z=%.2f g", accel_x, accel_y, accel_z);
            ESP_LOGI(TAG, "Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s", gyro_x, gyro_y, gyro_z);
           
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before next data read
        }
    } else {
        ESP_LOGI(TAG, "Failed to connect with Wi-Fi, check your network credentials\n");
    }

    // De-initialize I2C
    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}


  



