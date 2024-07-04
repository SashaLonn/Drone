#define I2C_MASTER_SCL_IO           4      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           3    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_FREQ_HZ          400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define MPU6050_SENSOR_ADDR          0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_ACCEL_XOUT_H_REG_ADDR 0x3B        /*!< X_value_H */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7
#define MPU6050_ACCEL_YOUT_H_REG_ADDR  0x3D        /*!< Y_value_H */
#define MPU6050_ACCEL_ZOUT_H_REG_ADDR  0x3F         /*!< Z_value_H */

#define MPU6050_GYRO_XOUT_H_REG_ADDR  0x43        /*!<Gyro X_value_H */
#define MPU6050_GYRO_YOUT_H_REG_ADDR  0x45        /*!<Gyro Y_value_H */
#define MPU6050_GYRO_ZOUT_H_REG_ADDR  0x47        /*!<Gyro Z_value_H */

// Constants for PID parameters
#define KP 1.0    // Proportional gain
#define KI 0.0    // Integral gain
#define KD 0.0    // Derivative gain

// PID limits
#define PID_OUTPUT_MIN -100.0
#define PID_OUTPUT_MAX 100.0

// PID sample time (milliseconds)
#define PID_SAMPLE_TIME_MS 10

// Target angles (adjust as needed)
#define TARGET_ROLL 0.0
#define TARGET_PITCH 0.0

#define ACCEL_SENSITIVITY 16384.0  // Sensitivity for ±2g full scale
#define GYRO_SENSITIVITY 131.0     // Sensitivity for ±250°/s full scale


// PWM pinnar för motorer
#define MOTOR1_PWM_GPIO    13  // Exempel: Använd GPIO 25 för PWM till motor 1
#define MOTOR1_PWM_CHANNEL  LEDC_CHANNEL_0
#define MOTOR_PWM_FREQ      5000 // PWM frekvens i Hz

#define MOTOR2_PWM_GPIO  12
#define MOTOR2_PWM_CHANNEL LEDC_CHANNEL_1


